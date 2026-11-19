# Copyright 2019 The OpenRadar Authors. All Rights Reserved.
# Copyright 2021 GaoWeifan. All Rights Reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

# _*_coding:utf-8_*_
"""
@File       : API_Config_FPGA.py
@Project:   : 
@Time       : 2022/12/6-10:54
@Author     : Long  Fan
@Software   : PyCharm
@License    : (C) Copyright 2021-2031, NanJing University

@Last Modify Time      @Version         @Description
-----------------      ---------        ------------
2022/12/6-10:54          1.0             None
"""

import codecs
import socket
import struct
import json
import math
from enum import Enum

import numpy as np
import fpga_udp

logTAG= "API_Config_DCA1000_FPGA  "

class CMD(Enum):
    RESET_FPGA_CMD_CODE = '0100'
    RESET_AR_DEV_CMD_CODE = '0200'
    CONFIG_FPGA_GEN_CMD_CODE = '0300'
    CONFIG_EEPROM_CMD_CODE = '0400'
    RECORD_START_CMD_CODE = '0500'
    RECORD_STOP_CMD_CODE = '0600'
    PLAYBACK_START_CMD_CODE = '0700'
    PLAYBACK_STOP_CMD_CODE = '0800'
    SYSTEM_CONNECT_CMD_CODE = '0900'
    SYSTEM_ERROR_CMD_CODE = '0a00'
    CONFIG_PACKET_DATA_CMD_CODE = '0b00'
    CONFIG_DATA_MODE_AR_DEV_CMD_CODE = '0c00'
    INIT_FPGA_PLAYBACK_CMD_CODE = '0d00'
    READ_FPGA_VERSION_CMD_CODE = '0e00'

    def __str__(self):
        return str(self.value)


# Global variables hold test configurations
# Set one time per test by read_config() function when read configuration parameters from test profile file
# Used by parser processing
CFG_PARAMS = {}

numOfChirps_buf = []
numLoops_buf = []
freqSlope_buf = []
numAdcSamples_buf = []
adcSampleRate_buf = []
profileIdx_buf = []
SigImgNumSlices_buf = []
RxSatNumSlices_buf = []
chanIdx_buf = []

lvdsCfg_headerEn_buf = []
lvdsCfg_dataFmt_buf = []
lvdsCfg_userBufEn_buf = []

Raw_file_numSubframes = 0
Raw_file_subframeIdx_buf = []
Raw_file_sessionFlag = ""

ADC_file_numSubframes = 0
ADC_file_subframeIdx_buf = []
ADC_file_sessionFlag = ""

CC9_file_numSubframes = 0
CC9_file_subframeIdx_buf = []
CC9_file_sessionFlag = ""

# Definations for test pass/fail or not_apply/don't care
NOT_APPLY = -1
TC_PASS = 0
TC_FAIL = 1

# MESSAGE = codecs.decode(b'5aa509000000aaee', 'hex')
CONFIG_HEADER = '5aa5'
HEADER_Num = 0xa55a
CONFIG_STATUS = '0000'
STATUS_STR = {0: 'success', 1: 'failed'}
CONFIG_FOOTER = 'aaee'
FOOTER_Num = 0xeeaa

ADC_PARAMS = {'chirps': 128,  # 32
              'rx': 4,
              'tx': 1,
              'samples': 64,
              'IQ': 2,
              'bytes': 2}
# STATIC
MAX_PACKET_SIZE = 4096
BYTES_IN_PACKET = 1456  # Data in payload per packet from FPGA
BYTES_OF_PACKET = 1466  # payload size per packet from FPGA
MAX_BYTES_PER_PACKET = 1470  # Maximum bytes in the data packet
FPGA_CLK_CONVERSION_FACTOR = 1000  # Record packet delay clock conversion factor
FPGA_CLK_PERIOD_IN_NANO_SEC = 8  # Record packet delay clock period in ns
VERSION_BITS_DECODE = 0x7F  # Version bits decode
VERSION_NUM_OF_BITS = 7  # Number of bits required for version
PLAYBACK_BIT_DECODE = 0x4000  # Playback FPGA bitfile identifier bit
# DYNAMIC
BYTES_IN_FRAME = (ADC_PARAMS['chirps'] * ADC_PARAMS['rx'] * ADC_PARAMS['tx'] *
                  ADC_PARAMS['IQ'] * ADC_PARAMS['samples'] * ADC_PARAMS['bytes'])

BYTES_IN_FRAME_CLIPPED = (BYTES_IN_FRAME // BYTES_IN_PACKET) * BYTES_IN_PACKET

PACKETS_IN_FRAME = BYTES_IN_FRAME / BYTES_IN_PACKET

PACKETS_IN_FRAME_CLIPPED = BYTES_IN_FRAME // BYTES_IN_PACKET
UINT16_IN_PACKET = BYTES_IN_PACKET // 2
UINT16_IN_FRAME = BYTES_IN_FRAME // 2


class DCA1000:
    """Software interface to the DCA1000 EVM board via ethernet.

    Attributes:
        static_ip (str): IP to receive data from the FPGA
        adc_ip (str): IP to send configuration commands to the FPGA
        data_port (int): Port that the FPGA is using to send data
        config_port (int): Port that the FPGA is using to read configuration commands from


    General steps are as follows:
        1. Power cycle DCA1000 and XWR1xxx sensor
        2. Open mmWaveStudio and setup normally until tab SensorConfig or use lua script
        3. Make sure to connect mmWaveStudio to the board via ethernet
        4. Start streaming data
        5. Read in frames using class

    Examples:
        >>> dca = DCA1000()
        >>> adc_data = dca.read(timeout=.1)
        >>> frame = dca.organize(adc_data, 128, 4, 256)

    """

    def __init__(self, static_ip='192.168.33.30', adc_ip='192.168.33.180',
                 data_port=4098, config_port=4096):
        # Save network data
        self.static_ip = static_ip
        self.adc_ip = adc_ip
        self.data_port = data_port
        self.config_port = config_port

        # Create configuration and data destinations
        self.cfg_dest = (adc_ip, config_port)
        self.cfg_recv = (static_ip, config_port)
        self.data_recv = (static_ip, data_port)


        # Create sockets
        self.config_socket = socket.socket(socket.AF_INET,
                                           socket.SOCK_DGRAM,
                                           socket.IPPROTO_UDP)
        self.data_socket = socket.socket(socket.AF_INET,
                                         socket.SOCK_DGRAM,
                                         socket.IPPROTO_UDP)
        
        self.data_socket.setsockopt(socket.SOL_SOCKET,
                                    socket.SO_RCVBUF,
                                    512 * 1024 * 1024)

        # Bind data socket to fpga
        self.data_socket.bind(self.data_recv)

        # Bind config socket to fpga
        self.config_socket.bind(self.cfg_recv)
        

        self.data = []
        self.packet_count = []
        self.byte_count = []

        self.frame_buff = []

        self.curr_buff = None
        self.last_frame = None

        self.lost_packets = None

        real_buf = self.data_socket.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        print(f"[DCA1000] UDP receive buffer: {real_buf / 1024 / 1024:.1f} MB")
        

        
    def refresh_parameter(self):
        global BYTES_IN_FRAME
        global BYTES_IN_FRAME_CLIPPED
        global PACKETS_IN_FRAME
        global PACKETS_IN_FRAME_CLIPPED
        global UINT16_IN_PACKET
        global UINT16_IN_FRAME

        # 引入已经解析好的子帧参数缓存列表
        global numAdcSamples_buf
        global numLoops_buf
        global CFG_PARAMS

        # DYNAMIC
        # 1. 检查是否为多子帧级联模式 (Advanced Frame)
        if CFG_PARAMS.get('numSubframes', 1) > 1:
            total_bytes = 0
            # 遍历所有子帧，累加每个子帧的数据量
            for sf_idx in range(CFG_PARAMS['numSubframes']):
                # 如果列表长度不够，为了防越界，取最后一个元素（通常情况下解析是对齐的）
                samples = numAdcSamples_buf[sf_idx] if sf_idx < len(numAdcSamples_buf) else numAdcSamples_buf[-1]
                loops = numLoops_buf[sf_idx] if sf_idx < len(numLoops_buf) else numLoops_buf[-1]
                
                # 累加当前子帧字节数: loops * rx * tx * IQ(2) * samples * bytes(2)
                subframe_bytes = (loops * ADC_PARAMS['rx'] * ADC_PARAMS['tx'] * ADC_PARAMS['IQ'] * samples * ADC_PARAMS['bytes'])
                total_bytes += subframe_bytes
            
            BYTES_IN_FRAME = total_bytes
        else:
            # 2. 传统的单帧模式 (Legacy Frame) 保持原样
            BYTES_IN_FRAME = (ADC_PARAMS['chirps'] * ADC_PARAMS['rx'] * ADC_PARAMS['tx'] *
                              ADC_PARAMS['IQ'] * ADC_PARAMS['samples'] * ADC_PARAMS['bytes'])

        # 3. 后续打包参数计算保持不变
        BYTES_IN_FRAME_CLIPPED = (BYTES_IN_FRAME // BYTES_IN_PACKET) * BYTES_IN_PACKET
        PACKETS_IN_FRAME = BYTES_IN_FRAME / BYTES_IN_PACKET
        PACKETS_IN_FRAME_CLIPPED = BYTES_IN_FRAME // BYTES_IN_PACKET
        UINT16_IN_PACKET = BYTES_IN_PACKET // 2
        UINT16_IN_FRAME = BYTES_IN_FRAME // 2
        
        
    # def refresh_parameter(self):
    #     global BYTES_IN_FRAME
    #     global BYTES_IN_FRAME_CLIPPED
    #     global PACKETS_IN_FRAME
    #     global PACKETS_IN_FRAME_CLIPPED
    #     global UINT16_IN_PACKET
    #     global UINT16_IN_FRAME
    #     # DYNAMIC
    #     BYTES_IN_FRAME = (ADC_PARAMS['chirps'] * ADC_PARAMS['rx'] * ADC_PARAMS['tx'] *
    #                       ADC_PARAMS['IQ'] * ADC_PARAMS['samples'] * ADC_PARAMS['bytes'])
    #     BYTES_IN_FRAME_CLIPPED = (BYTES_IN_FRAME // BYTES_IN_PACKET) * BYTES_IN_PACKET
    #     PACKETS_IN_FRAME = BYTES_IN_FRAME / BYTES_IN_PACKET
    #     PACKETS_IN_FRAME_CLIPPED = BYTES_IN_FRAME // BYTES_IN_PACKET
    #     UINT16_IN_PACKET = BYTES_IN_PACKET // 2
    #     UINT16_IN_FRAME = BYTES_IN_FRAME // 2

    @staticmethod
    def read_config(config_file_name, TXBF=False):
        """!
        This function read config from test profile file and fills up global variables to contain the configuration

            @param config_file_name : test config profile file name
            @return None
        """
        global CFG_PARAMS
        global ADC_PARAMS

        global numOfChirps_buf
        global numLoops_buf
        global numAdcSamples_buf
        global adcSampleRate_buf
        global freqSlope_buf
        global profileIdx_buf
        global SigImgNumSlices_buf
        global RxSatNumSlices_buf
        global chanIdx_buf

        global lvdsCfg_headerEn_buf
        global lvdsCfg_dataFmt_buf
        global lvdsCfg_userBufEn_buf

        global Raw_file_numSubframes
        global Raw_file_subframeIdx_buf
        global Raw_file_sessionFlag

        global ADC_file_numSubframes
        global ADC_file_subframeIdx_buf
        global ADC_file_sessionFlag

        global CC9_file_numSubframes
        global CC9_file_subframeIdx_buf
        global CC9_file_sessionFlag

        CFG_PARAMS = {}
        ADC_PARAMS = {}
        numOfChirps_buf = []
        numLoops_buf = []
        numAdcSamples_buf = []
        adcSampleRate_buf = []
        freqSlope_buf = []
        profileIdx_buf = []
        SigImgNumSlices_buf = []
        RxSatNumSlices_buf = []
        chanIdx_buf = []
        lvdsCfg_headerEn_buf = []
        lvdsCfg_dataFmt_buf = []
        lvdsCfg_userBufEn_buf = []
        Raw_file_subframeIdx_buf = []
        ADC_file_subframeIdx_buf = []
        CC9_file_subframeIdx_buf = []

        config = open(config_file_name, 'r')

        for line in config:
            # print("**** line from config file: \n" + line)
            List = line.split()

            if 'channelCfg' in line:
                CFG_PARAMS['rxAntMask'] = int(List[1])
                CFG_PARAMS['txAntMask'] = int(List[2])
            if 'adcCfg' in line:
                CFG_PARAMS['dataSize'] = int(List[1])
                CFG_PARAMS['dataType'] = int(List[2])
            if 'adcbufCfg' in line:
                CFG_PARAMS['chirpMode'] = int(List[5])

            if 'profileCfg' in line:
                profileIdx_buf.append(int(List[1]))
                ADC_PARAMS['startFreq'] = float(List[2])
                freqSlope_buf.append(float(List[8]))
                numAdcSamples_buf.append(int(List[10]))
                adcSampleRate_buf.append(int(List[11]))
                idleTime = float(List[3])
                rampEndTime = float(List[5])
                ADC_PARAMS['idleTime'] = idleTime
                ADC_PARAMS['adc_valid_start_time'] = float(List[4])
                ADC_PARAMS['rampEndTime'] = rampEndTime
                ADC_PARAMS['freq_slope'] = freqSlope_buf[0]
                ADC_PARAMS['txStartTime'] = float(List[9])
                ADC_PARAMS['samples'] = numAdcSamples_buf[0]
                ADC_PARAMS['sample_rate'] = adcSampleRate_buf[0]

            if 'frameCfg' in line:
                CFG_PARAMS['chirpStartIdx'] = int(List[1])
                CFG_PARAMS['chirpEndIdx'] = int(List[2])
                numOfChirps_buf.append(CFG_PARAMS['chirpEndIdx'] - CFG_PARAMS['chirpStartIdx'] + 1)
                numLoops_buf.append(int(List[3]))
                CFG_PARAMS['numSubframes'] = 1
                ADC_PARAMS['chirps'] = numLoops_buf[0]
                ADC_PARAMS['frame_periodicity'] = float(List[5])

            if 'advFrameCfg' in line:
                CFG_PARAMS['numSubframes'] = int(List[1])
            if 'subFrameCfg' in line:
                numOfChirps_buf.append(int(List[4]))
                numLoops_buf.append(int(List[5]))
                ADC_PARAMS['chirps'] = numLoops_buf[0]

            if 'lvdsStreamCfg' in line:
                lvdsCfg_headerEn_buf.append(int(List[2]))
                lvdsCfg_dataFmt_buf.append(int(List[3]))
                lvdsCfg_userBufEn_buf.append(int(List[4]))

            if 'CQSigImgMonitor' in line:
                SigImgNumSlices_buf.append(int(List[2]))

            if 'CQRxSatMonitor' in line:
                RxSatNumSlices_buf.append(int(List[4]))

        config.close()

        ####################################
        ######## Parser rxAnt config #######
        ####################################
        rxAntMask = CFG_PARAMS['rxAntMask']

        rxChanEn = []
        rxChanEn.append(rxAntMask & 1)
        rxChanEn.append((rxAntMask >> 1) & 1)
        rxChanEn.append((rxAntMask >> 2) & 1)
        rxChanEn.append((rxAntMask >> 3) & 1)
        # print(rxChanEn)

        txAntMask = CFG_PARAMS['txAntMask']
        # print(rxAntMask,txAntMask)
        txChanEn = []
        txChanEn.append(txAntMask & 1)
        txChanEn.append((txAntMask >> 1) & 1)
        txChanEn.append((txAntMask >> 2) & 1)
        # print(txChanEn)

        numRxChan = 0
        chanIdx_buf = []
        for chanIdx in range(4):
            if rxChanEn[chanIdx] == 1:
                chanIdx_buf.append(chanIdx)
                numRxChan = numRxChan + 1
        CFG_PARAMS['numRxChan'] = numRxChan

        numTxChan = 0
        for chanIdx in range(3):
            if txChanEn[chanIdx] == 1:
                numTxChan = numTxChan + 1
        if (TXBF):
            CFG_PARAMS['numTxChan'] = 1
        else:
            CFG_PARAMS['numTxChan'] = numTxChan
        ####################################
        ######## Parser lvds config ########
        ####################################
        Raw_file_numSubframes = 0
        Raw_file_subframeIdx_buf = []
        Raw_file_sessionFlag = ""

        ADC_file_numSubframes = 0
        ADC_file_subframeIdx_buf = []
        ADC_file_sessionFlag = ""

        CC9_file_numSubframes = 0
        CC9_file_subframeIdx_buf = []
        CC9_file_sessionFlag = ""

        # Based on the 1st subframe's lvdsStreamCfg CLI (headerEn, dataFmt and userBufEn)

        # if the 1st subframe has no header (headerEn = 0):
        # > in this case, HW session ADC only (dataFmt = 1) and no SW session (userBufEn = 0) is the only valid configuration combination.
        # > <prefix>_Raw_<x>.bin is generated to record HW session of the 1st subframe.
        # > in advanced subframe case, rest 3 subframes must have same lvdsStreamCfg as the 1st subframe, and record to <prefix>_Raw_<x>.bin as well.

        # if the 1st subframe has header (headerEn = 1) and HW session is ADC only (dataFmt = 1) or CP+ADC+CQ (dataFmt = 4):
        # > <prefix>_hdr_0ADC_<x>.bin is generated to record HW session of the 1st subframe. in advanced subframe case if any of rest 3 subfrmes has HW session, will be recorded to <prefix>_hdr_0ADC_<x>.bin as well.
        # > <prefix>_hdr_0CC9_<x>.bin will be generated to record SW session if any subframes has SW session (userBufEn = 1).

        # if the 1st subframe has header (headerEn = 1) and no HW session (dataFmt = 0):
        # > in this case, the 1st subframe must have SW session (userBufEn = 1)
        # > <prefix>_hdr_0ADC_<x>.bin is generated to record SW session of the 1st subframe. In advanced subframe case if any of rest 3 subframes has SW session, will be recorded to <prefix>_hdr_0ADC_<x>.bin as well.
        # > in advanced subframe case <prefix>_hdr_0CC9_<x>.bin will be generated to record HW session if any of rest 3 subframes has HW session (dataFmt = 1 or dataFmt = 4).

        CFG_PARAMS['datacard_dataLoggingMode'] = "multi"
        if lvdsCfg_headerEn_buf[0] == 0:
            CFG_PARAMS['datacard_dataLoggingMode'] = "raw"

        if lvdsCfg_headerEn_buf[0] == 0:
            if lvdsCfg_dataFmt_buf[0] == 1 and lvdsCfg_userBufEn_buf[0] == 0:
                if CFG_PARAMS['datacard_dataLoggingMode'] == "raw":
                    # Raw file
                    Raw_file_numSubframes = Raw_file_numSubframes + 1
                    Raw_file_subframeIdx_buf.append(0)
                    Raw_file_sessionFlag = "HW"
                elif CFG_PARAMS['datacard_dataLoggingMode'] == "multi":
                    returen_value = TC_FAIL
                    print("Error: no header can not be in multi mode!")
                else:
                    returen_value = TC_FAIL
                    print("Error: Undefined CFG_PARAMS['datacard_dataLoggingMode']!")
            else:
                returen_value = TC_FAIL
                print("Error: Invalid lvdsStreamCfg")
        elif lvdsCfg_headerEn_buf[0] == 1:
            if lvdsCfg_dataFmt_buf[0] == 1 or lvdsCfg_dataFmt_buf[0] == 4:  # 1:ADC 4:CP+ADC+CQ
                ADC_file_sessionFlag = "HW"
                CC9_file_sessionFlag = "SW"
                ADC_file_numSubframes = ADC_file_numSubframes + 1
                ADC_file_subframeIdx_buf.append(0)
                if lvdsCfg_userBufEn_buf[0] == 1:
                    CC9_file_numSubframes = CC9_file_numSubframes + 1
                    CC9_file_subframeIdx_buf.append(0)
            elif lvdsCfg_dataFmt_buf[0] == 0:  # no ADC no HW
                ADC_file_sessionFlag = "SW"
                CC9_file_sessionFlag = "HW"
                if lvdsCfg_userBufEn_buf[0] == 1:
                    ADC_file_numSubframes = ADC_file_numSubframes + 1
                    ADC_file_subframeIdx_buf.append(0)
                else:
                    returen_value = TC_FAIL
                    print("Error: subframe 0 has no HW and SW")
            else:
                print("subframe has a invalid dataFmt config")
        else:
            returen_value = TC_FAIL
            print("Error: Invalid lvdsCfg_headerEn_buf[0]")

        # Rest of 3 subframes if advanced subframe case
        for subframeIdx in range(1, CFG_PARAMS['numSubframes']):
            # [新增修复]：如果配置中使用了 -1 (长度为1)，则始终读取索引0；否则按当前子帧索引读取
            fmt_idx = 0 if len(lvdsCfg_dataFmt_buf) == 1 else subframeIdx
            buf_idx = 0 if len(lvdsCfg_userBufEn_buf) == 1 else subframeIdx

            if lvdsCfg_dataFmt_buf[fmt_idx] == 1 or lvdsCfg_dataFmt_buf[fmt_idx] == 4:  # 1:ADC 4:CP+ADC+CQ
                if ADC_file_sessionFlag == "HW":
                    ADC_file_numSubframes = ADC_file_numSubframes + 1
                    ADC_file_subframeIdx_buf.append(subframeIdx)
                if CC9_file_sessionFlag == "HW":
                    CC9_file_numSubframes = CC9_file_numSubframes + 1
                    CC9_file_subframeIdx_buf.append(subframeIdx)
            if lvdsCfg_userBufEn_buf[buf_idx] == 1:
                if ADC_file_sessionFlag == "SW":
                    ADC_file_numSubframes = ADC_file_numSubframes + 1
                    ADC_file_subframeIdx_buf.append(subframeIdx)
                if CC9_file_sessionFlag == "SW":
                    CC9_file_numSubframes = CC9_file_numSubframes + 1
                    CC9_file_subframeIdx_buf.append(subframeIdx)
    
        # # Rest of 3 subframes if advanced subframe case
        # for subframeIdx in range(1, CFG_PARAMS['numSubframes']):
        #     # 如果列表里只有一个元素（说明用了 -1 通配所有子帧），就永远取第 0 个；否则按子帧索引取
        #     fmt_idx = 0 if len(lvdsCfg_dataFmt_buf) == 1 else subframeIdx
        #     if lvdsCfg_dataFmt_buf[fmt_idx] == 1 or lvdsCfg_dataFmt_buf[fmt_idx] == 4:  # 1:ADC 4:CP+ADC+CQ
        #     # if lvdsCfg_dataFmt_buf[subframeIdx] == 1 or lvdsCfg_dataFmt_buf[subframeIdx] == 4:  # 1:ADC 4:CP+ADC+CQ
        #         if ADC_file_sessionFlag == "HW":
        #             ADC_file_numSubframes = ADC_file_numSubframes + 1
        #             ADC_file_subframeIdx_buf.append(subframeIdx)
        #         if CC9_file_sessionFlag == "HW":
        #             CC9_file_numSubframes = CC9_file_numSubframes + 1
        #             CC9_file_subframeIdx_buf.append(subframeIdx)
        #     if lvdsCfg_userBufEn_buf[subframeIdx] == 1:
        #         if ADC_file_sessionFlag == "SW":
        #             ADC_file_numSubframes = ADC_file_numSubframes + 1
        #             ADC_file_subframeIdx_buf.append(subframeIdx)
        #         if CC9_file_sessionFlag == "SW":
        #             CC9_file_numSubframes = CC9_file_numSubframes + 1
        #             CC9_file_subframeIdx_buf.append(subframeIdx)

        ADC_PARAMS['rx'] = CFG_PARAMS['numRxChan']
        ADC_PARAMS['tx'] = CFG_PARAMS['numTxChan']
        ADC_PARAMS['IQ'] = 2
        ADC_PARAMS['bytes'] = 2

        '''
        For HW data, the inter-chirp duration should be sufficient to stream out the desired amount of data. 
        For example, if the HW data-format is ADC and HSI header is enabled, then the total amount of data 
        generated per chirp is:(numAdcSamples * numRxChannels * 4 (size of complex sample) + 
        52 [sizeof(HSIDataCardHeader_t) + sizeof(HSISDKHeader_t)] ) 
        rounded up to multiples of 256 [=sizeof(HSIHeader_t)] bytes.
        
        The chirp time Tc in us = idle time + ramp end time in the profile configuration. 
        For n-lane LVDS with each lane at a maximum of B Mbps,
        maximum number of bytes that can be send per chirp = Tc * n * B / 8 
        which should be greater than the total amount of data generated per chirp i.e
        Tc * n * B / 8 >= round-up(numAdcSamples * numRxChannels * 4 + 52, 256).
        E.g if n = 2, B = 600 Mbps, idle time = 7 us, ramp end time = 44 us, numAdcSamples = 512, numRxChannels = 4, 
        then 7650 >= 8448 is violated so this configuration will not work. 
        If the idle-time is doubled in the above example, then we have 8700 > 8448, so this configuration will work.

        For SW data, the number of bytes to transmit each sub-frame/frame is:
        52 [sizeof(HSIDataCardHeader_t) + sizeof(HSISDKHeader_t)] + sizeof(MmwDemo_LVDSUserDataHeader_t) [=8] +
        number of detected objects
        (Nd) * { sizeof(DPIF_PointCloudCartesian_t) [=16] + sizeof(DPIF_PointCloudSideInfo_t) [=4] }
        rounded up to multiples of 256 [=sizeof(HSIHeader_t)] bytes.
        or X = round-up(60 + Nd * 20, 256). So the time to transmit this data will be X * 8 / (n*B) us. 
        The maximum number of objects (Ndmax) that can be detected is defined in the DPC (DPC_OBJDET_MAX_NUM_OBJECTS). 
        So if Ndmax = 500, then time to transmit SW data is 68 us. 
        Because we parallelize this transmission with the much slower UART transmission, 
        and because UART transmission is also sending at least the same amount of information as the LVDS, 
        the LVDS transmission time will not add any burdens on the processing budget beyond the overhead of 
        reconfiguring and activating the CBUFF session (this overhead is likely bigger than the time to transmit).

        The total amount of data to be transmitted in a HW or SW packet must be greater than the minimum 
        required by CBUFF, which is 64 bytes or 32 CBUFF Units 
        (this is the definition CBUFF_MIN_TRANSFER_SIZE_CBUFF_UNITS in the CBUFF driver implementation). 
        If this threshold condition is violated, 
        the CBUFF driver will return an error during configuration 
        and the demo will generate a fatal exception as a result. 
        When HSI header is enabled, the total transfer size is ensured to be at least 256 bytes, 
        which satisfies the minimum. 
        If HSI header is disabled, for the HW session, this means that numAdcSamples * numRxChannels * 4 >= 64. 
        Although mmwavelink allows minimum number of ADC samples to be 2, the demo is supported for numAdcSamples >= 64. 
        So HSI header is not required to be enabled for HW only case. 
        But if SW session is enabled, without the HSI header, the bytes in each packet will be 8 + Nd * 20. 
        So for frames/sub-frames where Nd < 3, the demo will generate exception. 
        Therefore HSI header must be enabled if SW is enabled, this is checked in the CLI command validation.
        '''
        LVDSDataSizePerChirp = numAdcSamples_buf[0] * CFG_PARAMS['numRxChan'] * ADC_PARAMS['IQ'] * ADC_PARAMS[
            'bytes'] + 52
        LVDSDataSizePerChirp = math.ceil(LVDSDataSizePerChirp / 256) * 256
        nlane = 2
        B = 600
        maxSendBytesPerChirp = (idleTime + rampEndTime) * nlane * B / 8
        # print(ADC_PARAMS)
        return (LVDSDataSizePerChirp, maxSendBytesPerChirp, ADC_PARAMS, CFG_PARAMS)

    @staticmethod
    def parser_raw_adc_data(raw_bin_file, config_file_name):
        """ parser raw adc .bin data
        Args:
            raw_bin_file: input raw adc bin data file
            config_file_name: config file

        Returns: [Frames, Chirps, TX, RX, Samples]
                 or [Frames, Chirps, VirtualAnts, Samples]

        """
        _, _, ADC_PARAMS, _ = DCA1000.read_config(config_file_name)

        adc_data = np.fromfile(raw_bin_file, dtype=np.int16)
        adc_data = np.reshape(adc_data, (-1, ADC_PARAMS['chirps'], ADC_PARAMS['tx'],
                                         ADC_PARAMS['rx'], ADC_PARAMS['samples'] // 2, ADC_PARAMS['IQ'], 2))

        adc_data = np.transpose(adc_data, (0, 1, 2, 3, 4, 6, 5))
        adc_data = np.reshape(adc_data, (-1, ADC_PARAMS['chirps'], ADC_PARAMS['tx'],
                                         ADC_PARAMS['rx'], ADC_PARAMS['samples'], ADC_PARAMS['IQ']))

        # [Frames, Chirps, TX, RX, Samples]
        adc_data = (1j * adc_data[:, :, :, :, :, 0] + adc_data[:, :, :, :, :, 1]).astype(np.complex64)

        # [Frames * Chirps * VirtualAnts * Samples]
        adc_data = np.reshape(adc_data[:, :, 0:ADC_PARAMS['tx'], :, :],
                              (-1, ADC_PARAMS['chirps'], ADC_PARAMS['tx'] * ADC_PARAMS['rx'], ADC_PARAMS['samples']))
        return adc_data

    def __del__(self):
        # print("Stop stream: ",self.stream_stop())
        self.close()
        print("Close socket: success")

    def configure(self, FPGAjsonFilePath, IWRcfgFilePath):
        """Initializes and connects to the FPGA

        Returns:
            None

        """
        print("=====================DCA1000EVM FPGA Configuration Start...")
        LVDSDataSizePerChirp_l, maxSendBytesPerChirp_l, ADC_PARAMS_l, CFG_PARAMS_l = self.read_config(IWRcfgFilePath)
        self.refresh_parameter()
        print(ADC_PARAMS_l)
        print("\n\n")
        print(CFG_PARAMS_l)
        print("LVDSDataSizePerChirp: {}, must <= maxSendBytesPerChirp: {}".format(LVDSDataSizePerChirp_l, maxSendBytesPerChirp_l))
        print("System connection check: {}\n".format(self.sys_alive_check()))
        print("FPGA Version: {}\n".format(self.read_fpga_version()))
        print("Config fpga: {}\n".format(self.config_fpga(FPGAjsonFilePath)))
        print("Config record packet delay: {}\n".format(self.config_record(FPGAjsonFilePath)))
        print("========================DCA1000EVM FPGA Configuration Stop...\n")

        return ADC_PARAMS_l, CFG_PARAMS_l

    def close(self):
        """Closes the sockets that are used for receiving and sending data

        Returns:
            None

        """
        self.data_socket.close()
        self.config_socket.close()

    def read(self, timeout=1):
        """ Read in a single packet via UDP

        Args:
            timeout (float): Time to wait for packet before moving on

        Returns:
            Full frame as array if successful, else None

        """
        # Configure
        self.data_socket.settimeout(timeout)

        # Frame buffer
        ret_frame = np.zeros(UINT16_IN_FRAME, dtype=np.uint16)
        # print(BYTES_IN_FRAME, BYTES_IN_FRAME_CLIPPED, BYTES_IN_PACKET)

        # Wait for start of next frame
        while True:
            packet_num, byte_count, packet_data = self._read_data_packet()
            if byte_count % BYTES_IN_FRAME_CLIPPED == 0:
                packets_read = 1
                ret_frame[0:UINT16_IN_PACKET] = packet_data
                break

        # Read in the rest of the frame
        while True:
            packet_num, byte_count, packet_data = self._read_data_packet()
            packets_read += 1

            if byte_count % BYTES_IN_FRAME_CLIPPED == 0:
                self.lost_packets = PACKETS_IN_FRAME_CLIPPED - packets_read
                return ret_frame

            curr_idx = ((packet_num - 1) % PACKETS_IN_FRAME_CLIPPED)
            try:
                ret_frame[curr_idx * UINT16_IN_PACKET:(curr_idx + 1) * UINT16_IN_PACKET] = packet_data
            except:
                pass

            if packets_read > PACKETS_IN_FRAME_CLIPPED:
                packets_read = 0

    def postProcPacket(self, recvQueue, maxPacketNum):
        # global firstPacketNum
        # global receivedPacketNum
        data = recvQueue.pop(0)
        packet_data = np.frombuffer(data[10:], dtype=np.uint16)
        firstPacketNum = struct.unpack('<1l', data[:4])[0]
        
        # print(logTAG + f"First Packet ID - {firstPacketNum}")

        receivedPacketNum = [firstPacketNum]
        receivedData = np.zeros(UINT16_IN_PACKET * maxPacketNum, dtype=np.uint16)
        receivedData[0:UINT16_IN_PACKET] = packet_data
        while (recvQueue):
            data = recvQueue.pop(0)
            packet_data = np.frombuffer(data[10:], dtype=np.uint16)
            packetNum = struct.unpack('<1l', data[:4])[0]
            idx = packetNum - firstPacketNum
            if (idx + 1 > maxPacketNum or idx < 1):
                continue
            receivedPacketNum.append(packetNum)
            receivedData[UINT16_IN_PACKET * idx:UINT16_IN_PACKET * (idx + 1)] = packet_data
        
        # print(logTAG + f"Last Packet ID - {packetNum}")

        return receivedData, firstPacketNum, receivedPacketNum

    def fastRead_fram0(self, numframes=1):
        packetNum = math.ceil(PACKETS_IN_FRAME * numframes)

        recvQueue = []
        for i in range(packetNum):
            data, addr = self.data_socket.recvfrom(MAX_BYTES_PER_PACKET)
            recvQueue.append(data)

        # print("all received, post processing packets...")

        receivedData, \
            firstPacketNum, \
            receivedPacketNum = self.postProcPacket(recvQueue, packetNum)
        
        databuf = receivedData[0:numframes * UINT16_IN_FRAME]
        
        # print("received packet num:%d,expected packet num:%d" % (len(receivedPacketNum), packetNum))

        return databuf

    # def fastRead_in_Cpp(self, numframes=1):
    #     packetNum = math.ceil(PACKETS_IN_FRAME * numframes)

    #     recvData = fpga_udp.read_data_udp(self.data_socket.fileno(), 
    #                                       packetNum, BYTES_OF_PACKET, 6)
    #     # print(logTAG + "all received, post processing packets...\n")

    #     recvData = np.reshape(recvData, (packetNum, BYTES_OF_PACKET))
    #     recvQueue = list(map(lambda x: bytes(x), recvData))

    #     receivedData, \
    #         firstPacketNum, \
    #         receivedPacketNum = self.postProcPacket(recvQueue, packetNum)
    #     databuf = receivedData[0:numframes * UINT16_IN_FRAME]

    #     # print(logTAG + "received packet num:%d,expected packet num:%d \n" % (len(receivedPacketNum), packetNum))

    #     return databuf
    
    def fastRead_in_Cpp(self, numframes=1):
        packetNum = math.ceil(PACKETS_IN_FRAME * numframes)

        recvData = fpga_udp.read_data_udp(
            self.data_socket.fileno(),
            packetNum,
            BYTES_OF_PACKET,
            6
        )

        valid_len = (recvData.size // BYTES_OF_PACKET) * BYTES_OF_PACKET
        if valid_len <= 0:
            raise RuntimeError("[DCA1000] No valid UDP packet received")

        if valid_len != recvData.size:
            print(f"[DCA1000] Warning: truncate incomplete UDP tail: "
                f"{recvData.size} -> {valid_len}")

        recvData = recvData[:valid_len]
        packetNum_real = valid_len // BYTES_OF_PACKET

        print(f"[DCA1000] expected packets: {packetNum}, "
            f"received packets: {packetNum_real}")

        recvData = np.reshape(recvData, (packetNum_real, BYTES_OF_PACKET))
        recvQueue = list(map(lambda x: bytes(x), recvData))

        receivedData, firstPacketNum, receivedPacketNum = self.postProcPacket(
            recvQueue,
            packetNum_real
        )

        databuf = receivedData[0:min(numframes * UINT16_IN_FRAME, receivedData.size)]

        return databuf


    def write_frames_to_file(self, filename="raw_data.bin", numframes=1):
        # data_buf = self.read()
        # for i in range(numframes-1):
        #     print("%d frame received"%(i+2))
        #     data_buf = np.append(data_buf, self.read())

        # data_buf = self.fastRead_fram0(numframes)
        data_buf = self.fastRead_in_Cpp(numframes)
        data_buf.tofile(filename)
        print("file saved to", filename)
        
    def set_capture_complete_callback(self, callback_func):
        """
        设置数据采集完成的回调函数
        :param callback_func: 上层传入的回调函数
        """
        self.capture_complete_callback = callback_func
            

    # def write_frames_to_file_until_stop(self, filename="raw_data.bin", chunk_frames=10):
    def write_frames_to_file_until_stop_bak(self, 
                                        filename="raw_data.bin", 
                                        chunk_frames=20, 
                                        stop_event=None,
                                        frame_callback=None,
                                        frame_period_s=None,
                                        radar_start_time=None,
                                        start_frame_idx=0):
        """
            Continuously capture DCA1000 UDP data until stop_event or Ctrl+C.

            Parameters
            ----------
            filename:
                Output raw ADC BIN file.

            chunk_frames:
                Number of frames requested from read_data_udp() per chunk.
                This is not the total number of captured frames.

            stop_event:
                Multiprocessing stop event.

            frame_callback:
                Called after each complete frame is written:

                    frame_callback(
                        frame_idx=...,
                        system_time=...,
                        relative_time=...,
                        frame_valid=...,
                        packet_sequence=...,
                    )

            frame_period_s:
                Radar frame period in seconds, e.g. 0.03333.

            radar_start_time:
                Shared Unix timestamp used by markers.csv and viewpoint.csv.

            start_frame_idx:
                Initial global frame index.

            Notes
            -----
            Since read_data_udp() returns packets in chunks without per-packet
            timestamps, the generated frame timestamps are radar-schedule
            timestamps rather than true UDP arrival timestamps:

                relative_time = frame_idx * frame_period_s
        """

        packetNum_per_chunk = math.ceil(PACKETS_IN_FRAME * chunk_frames)

        total_received_packets = 0
        total_saved_uint16 = 0
        total_saved_frames = 0
        total_chunks = 0

        global_first_pkt_id = None
        global_last_pkt_id = None
        prev_last_pkt_id = None
        estimated_lost_packets = 0

         # Preserve incomplete frame samples between chunks.
        pending_uint16 = np.empty(
            0,
            dtype=np.uint16,
        )

        next_frame_idx = int(start_frame_idx)

        # print("\n[DCA1000] Start infinite capture.")
        # print("[DCA1000] Press Ctrl+C or set stop_event to stop.")
        # print(f"[DCA1000] Output file: {filename}")
        # print(f"[DCA1000] Chunk frames: {chunk_frames}")
        # print(f"[DCA1000] Packets per chunk: {packets_per_chunk}")
        # print(f"[DCA1000] Frame period: {frame_period_s:.6f} s\n")


        try:
            with open(filename, "wb") as f:
                while True:
                    if stop_event is not None and stop_event.is_set():
                        print("[DCA1000] stop_event received. Stop capture.")
                        break
                    
                    chunk_receive_time = time.time()
                    
                    recvData = fpga_udp.read_data_udp(
                        self.data_socket.fileno(),
                        packetNum_per_chunk,
                        BYTES_OF_PACKET,
                        6)
                    if recv_data is None:
                        print("[DCA1000] Warning: read_data_udp returned None.")
                        continue

                    valid_len = (recvData.size // BYTES_OF_PACKET) * BYTES_OF_PACKET

                    if valid_len <= 0:
                        print("[DCA1000] Warning: no valid UDP packet in this chunk, continue...")
                        continue

                    if valid_len != recvData.size:
                        print(f"[DCA1000] Warning: truncate incomplete UDP tail: "
                            f"{recvData.size} -> {valid_len}")

                    recvData = recvData[:valid_len]
                    packetNum_real = valid_len // BYTES_OF_PACKET

                    recvData = np.reshape(recvData, (packetNum_real, BYTES_OF_PACKET))
                    recvQueue = list(map(lambda x: bytes(x), recvData))

                    received_packet_ids = []
                    for packet in recvQueue:
                        pkt_id = struct.unpack('<1L', packet[:4])[0]
                        received_packet_ids.append(pkt_id)

                    if len(received_packet_ids) == 0:
                        print("[DCA1000] Warning: empty packet id list, continue...")
                        continue

                    first_pkt_id = min(received_packet_ids)
                    last_pkt_id = max(received_packet_ids)

                    if global_first_pkt_id is None:
                        global_first_pkt_id = first_pkt_id

                    global_last_pkt_id = last_pkt_id

                    # chunk 内部丢包估计
                    total_packets_in_chunk_seq = last_pkt_id - first_pkt_id + 1
                    received_unique_count = len(set(received_packet_ids))
                    chunk_lost = total_packets_in_chunk_seq - received_unique_count

                    # chunk 之间的 gap 丢包估计
                    gap_lost = 0
                    if prev_last_pkt_id is not None and first_pkt_id > prev_last_pkt_id + 1:
                        gap_lost = first_pkt_id - prev_last_pkt_id - 1

                    estimated_lost_packets += chunk_lost + gap_lost
                    prev_last_pkt_id = last_pkt_id

                    # ------------------------------------------------
                    # Packet reconstruction
                    # ------------------------------------------------
                    receivedData, firstPacketNum, receivedPacketNum = self.postProcPacket(
                        recvQueue,
                        packetNum_real
                    )

                    expected_uint16 = chunk_frames * UINT16_IN_FRAME
                    valid_uint16 = min(expected_uint16, receivedData.size)
                    databuf = receivedData[:valid_uint16]

                    databuf.tofile(f)
                    f.flush()

                    total_chunks += 1
                    total_received_packets += packetNum_real
                    total_saved_uint16 += valid_uint16

                    estimated_frames = total_saved_uint16 / UINT16_IN_FRAME

                    if total_chunks % 10 == 0:
                        total_stream_packets = (
                            global_last_pkt_id - global_first_pkt_id + 1
                            if global_first_pkt_id is not None and global_last_pkt_id is not None
                            else 0
                        )

                        loss_rate = (
                            estimated_lost_packets / total_stream_packets
                            if total_stream_packets > 0
                            else 0.0
                        )

                        # print(
                        #     f"[DCA1000] chunks={total_chunks}, "
                        #     f"received_packets={total_received_packets}, "
                        #     f"saved_uint16={total_saved_uint16}, "
                        #     f"estimated_frames={estimated_frames:.1f}, "
                        #     f"loss={loss_rate * 100:.3f}%"
                        # )

        except KeyboardInterrupt:
            print("\n[DCA1000] Manual stop received by Ctrl+C.")

        finally:
            estimated_frames = total_saved_uint16 / UINT16_IN_FRAME

            total_stream_packets = (
                global_last_pkt_id - global_first_pkt_id + 1
                if global_first_pkt_id is not None and global_last_pkt_id is not None
                else 0
            )

            loss_rate = (
                estimated_lost_packets / total_stream_packets
                if total_stream_packets > 0
                else 0.0
            )

            success = total_received_packets > 0

            if self.capture_complete_callback is not None:
                self.capture_complete_callback(
                    success=success,
                    filename=filename,
                    num_frames=int(estimated_frames),
                    packet_count=total_received_packets
                )

            print(f"\n========== Infinite Capture Report | File: {filename} ==========")
            print(f"Total chunks:             {total_chunks}")
            print(f"Total received packets:   {total_received_packets}")
            print(f"First packet ID:          {global_first_pkt_id}")
            print(f"Last packet ID:           {global_last_pkt_id}")
            print(f"Total packets in stream:  {total_stream_packets}")
            print(f"Estimated lost packets:   {estimated_lost_packets}")
            print(f"Estimated loss rate:      {loss_rate * 100:.3f} %")
            print(f"Saved uint16 samples:     {total_saved_uint16}")
            print(f"Estimated saved frames:   {estimated_frames:.1f}")
            print(f"Capture success flag:     {success}")
            print(f"===============================================================\n")

            print(f"[DCA1000] File saved to: {filename}")

    def write_frames_to_file_with_loss_rate_vbak(self, filename="raw_data.bin", numframes=1):
        """
        Read multiple frames via high-speed UDP capture, save to binary file,
        and calculate packet loss rate for the entire capture session.

        Args:
            filename (str): Output binary file path to save raw ADC data
            numframes (int): Number of frames to capture and write

        Returns:
            None
        """
        # Calculate total number of UDP packets required for requested frames
        packetNum = math.ceil(PACKETS_IN_FRAME * numframes)

        # Read raw UDP packets using the C++ accelerated interface
        recvData = fpga_udp.read_data_udp(self.data_socket.fileno(), packetNum, BYTES_OF_PACKET, 6)
        recvData = np.reshape(recvData, (packetNum, BYTES_OF_PACKET))
        recvQueue = list(map(lambda x: bytes(x), recvData))
        
        print(">"*70 + "\n")
        if self.capture_complete_callback is not None:
            self.capture_complete_callback(
                success=True,    
                filename=filename,     
                num_frames=numframes,  
                packet_count=packetNum 
            )
        print("All packets received, starting post-processing...[STOP COLLECTION]")
        print("<"*70 + "\n")
        

        # ===================== CORRECT Packet Loss Statistics =====================
        received_packet_ids = []
        for packet in recvQueue:
            pkt_id = struct.unpack('<1L', packet[:4])[0]
            received_packet_ids.append(pkt_id)

        if not received_packet_ids:
            print("Error: No packets received!")
            return

        # Get first & last packet ID (correct way to calculate total packets in sequence)
        first_pkt_id = min(received_packet_ids)
        last_pkt_id = max(received_packet_ids)
        total_packets_in_sequence = last_pkt_id - first_pkt_id + 1

        # Actual received packets (count unique IDs to avoid duplicate counting)
        received_unique_count = len(set(received_packet_ids))

        # Real packet loss calculation
        lost_count = total_packets_in_sequence - received_unique_count
        loss_rate = lost_count / total_packets_in_sequence if total_packets_in_sequence > 0 else 0.0

        # ===================== Reconstruct ADC Frame Data =====================
        receivedData, firstPacketNum, receivedPacketNum = self.postProcPacket(recvQueue, packetNum)
        databuf = receivedData[0:numframes * UINT16_IN_FRAME]

        # ===================== Write to Binary File =====================
        databuf.tofile(filename)

        # ===================== Print Loss Report =====================
        print(f"\n========== Packet Loss Report | File: {filename} ==========")
        print(f"First packet ID:         {first_pkt_id}")
        print(f"Last packet ID:          {last_pkt_id}")
        print(f"Total packets in stream: {total_packets_in_sequence}")
        print(f"Received unique packets: {received_unique_count}")
        print(f"Lost packets:            {lost_count}")
        print(f"Packet loss rate:        {loss_rate * 100:.3f} %")
        print(f"=============================================================\n")

        print(f"File successfully saved to: {filename}")
    

    def write_frames_to_file_until_stop(
                                        self,
                                        filename="raw_data.bin",
                                        chunk_frames=100,
                                        stop_event=None,
                                        frame_callback=None,
                                        frame_period_s=0.05,
                                        radar_start_time=None,
                                        start_frame_idx=0,):
        """
        Continuously capture DCA1000 UDP data until stop_event or Ctrl+C.

        Notes
        -----
        The timestamp of frame i is generated as:

            relative_time = frame_idx * frame_period_s
            system_time   = radar_start_time + relative_time

        This is the radar-schedule timestamp rather than the actual UDP
        arrival timestamp.
        """

        chunk_frames = int(chunk_frames)

        if chunk_frames <= 0:
            raise ValueError("chunk_frames must be positive.")

        if frame_period_s is None or frame_period_s <= 0:
            raise ValueError(
                "frame_period_s must be positive, "
                "for example 0.03333 for 30 FPS."
            )

        if radar_start_time is None or radar_start_time <= 0:
            raise ValueError(
                "radar_start_time must be a valid Unix timestamp."
            )

        packet_num_per_chunk = math.ceil(
            PACKETS_IN_FRAME * chunk_frames
        )

        total_received_packets = 0
        total_saved_uint16 = 0
        total_saved_frames = 0
        total_chunks = 0

        global_first_pkt_id = None
        global_last_pkt_id = None
        prev_last_pkt_id = None
        estimated_lost_packets = 0

        # 保存当前 chunk 中不足一帧的尾部数据
        pending_uint16 = np.empty(0, dtype=np.uint16)

        next_frame_idx = int(start_frame_idx)

        # print("\n[DCA1000] Start infinite capture.")
        # print("[DCA1000] Press Ctrl+C or set stop_event to stop.")
        # print(f"[DCA1000] Output file: {filename}")
        # print(f"[DCA1000] Chunk frames: {chunk_frames}")
        # print(f"[DCA1000] Packets per chunk:{packet_num_per_chunk}")
        # print(
        #     f"[DCA1000] Frame period: "
        #     f"{frame_period_s:.6f} s\n"
        # )

        try:
            with open(filename, "wb") as output_file:
                while True:
                    if (
                        stop_event is not None
                        and stop_event.is_set()
                    ):
                        print(
                            "[DCA1000] stop_event received. "
                            "Stop capture."
                        )
                        break

                    recv_data = fpga_udp.read_data_udp(
                        self.data_socket.fileno(),
                        packet_num_per_chunk,
                        BYTES_OF_PACKET,
                        6,
                    )

                    if recv_data is None:
                        print(
                            "[DCA1000] Warning: "
                            "read_data_udp returned None."
                        )
                        continue

                    recv_data = np.asarray(recv_data)

                    valid_len = (
                        recv_data.size // BYTES_OF_PACKET
                    ) * BYTES_OF_PACKET

                    if valid_len <= 0:
                        print(
                            "[DCA1000] Warning: no valid UDP "
                            "packet in this chunk."
                        )
                        continue

                    if valid_len != recv_data.size:
                        print(
                            "[DCA1000] Warning: truncate "
                            "incomplete UDP tail: "
                            f"{recv_data.size} -> {valid_len}"
                        )

                    recv_data = recv_data[:valid_len]

                    packet_num_real = (
                        valid_len // BYTES_OF_PACKET
                    )

                    recv_data = np.reshape(
                        recv_data,
                        (
                            packet_num_real,
                            BYTES_OF_PACKET,
                        ),
                    )

                    recv_queue = [
                        bytes(packet)
                        for packet in recv_data
                    ]

                    # ------------------------------------------------
                    # 1. Packet loss statistics
                    # ------------------------------------------------
                    received_packet_ids = []

                    for packet in recv_queue:
                        if len(packet) < 4:
                            continue

                        packet_id = struct.unpack(
                            "<1L",
                            packet[:4],
                        )[0]

                        received_packet_ids.append(
                            packet_id
                        )

                    if not received_packet_ids:
                        print(
                            "[DCA1000] Warning: empty packet "
                            "ID list."
                        )
                        continue

                    first_pkt_id = min(
                        received_packet_ids
                    )

                    last_pkt_id = max(
                        received_packet_ids
                    )

                    if global_first_pkt_id is None:
                        global_first_pkt_id = (
                            first_pkt_id
                        )

                    global_last_pkt_id = last_pkt_id

                    total_packets_in_chunk_seq = (
                        last_pkt_id
                        - first_pkt_id
                        + 1
                    )

                    received_unique_count = len(
                        set(received_packet_ids)
                    )

                    chunk_lost = max(
                        total_packets_in_chunk_seq
                        - received_unique_count,
                        0,
                    )

                    gap_lost = 0

                    if (
                        prev_last_pkt_id is not None
                        and first_pkt_id
                        > prev_last_pkt_id + 1
                    ):
                        gap_lost = (
                            first_pkt_id
                            - prev_last_pkt_id
                            - 1
                        )

                    estimated_lost_packets += (
                        chunk_lost + gap_lost
                    )

                    prev_last_pkt_id = last_pkt_id

                    # ------------------------------------------------
                    # 2. Reconstruct UDP packets
                    # ------------------------------------------------
                    (
                        received_data,
                        first_packet_num,
                        received_packet_num,
                    ) = self.postProcPacket(
                        recv_queue,
                        packet_num_real,
                    )

                    if received_data is None:
                        print(
                            "[DCA1000] Warning: "
                            "postProcPacket returned None."
                        )
                        continue

                    received_data = np.asarray(
                        received_data,
                        dtype=np.uint16,
                    ).reshape(-1)

                    if received_data.size == 0:
                        print(
                            "[DCA1000] Warning: no reconstructed "
                            "ADC samples."
                        )
                        continue

                    # ------------------------------------------------
                    # 3. Merge the remainder from the previous chunk
                    # ------------------------------------------------
                    if pending_uint16.size > 0:
                        frame_stream = np.concatenate(
                            (
                                pending_uint16,
                                received_data,
                            )
                        )
                    else:
                        frame_stream = received_data

                    complete_frame_num = (
                        frame_stream.size
                        // UINT16_IN_FRAME
                    )

                    complete_uint16 = (
                        complete_frame_num
                        * UINT16_IN_FRAME
                    )

                    complete_data = frame_stream[
                        :complete_uint16
                    ]

                    pending_uint16 = frame_stream[
                        complete_uint16:
                    ].copy()

                    # ------------------------------------------------
                    # 4. Write complete frames only
                    # ------------------------------------------------
                    if complete_frame_num > 0:
                        complete_data.tofile(
                            output_file
                        )

                        output_file.flush()

                        # 为每个完整写入的 frame 保存 timestamp
                        for _ in range(
                            complete_frame_num
                        ):
                            frame_idx = next_frame_idx

                            relative_time = (
                                frame_idx
                                * frame_period_s
                            )

                            system_time = (
                                radar_start_time
                                + relative_time
                            )

                            if frame_callback is not None:
                                frame_callback(
                                    frame_idx=frame_idx,
                                    system_time=system_time,
                                    relative_time=relative_time,
                                    frame_valid=True,
                                    packet_sequence=None,
                                )

                            next_frame_idx += 1

                        saved_uint16 = (
                            complete_frame_num
                            * UINT16_IN_FRAME
                        )

                        total_saved_uint16 += (
                            saved_uint16
                        )

                        total_saved_frames += (
                            complete_frame_num
                        )

                    total_chunks += 1
                    total_received_packets += (
                        packet_num_real
                    )

                    # ------------------------------------------------
                    # 5. Runtime status
                    # ------------------------------------------------
                    if total_chunks % 10 == 0:
                        if (
                            global_first_pkt_id
                            is not None
                            and global_last_pkt_id
                            is not None
                        ):
                            total_stream_packets = (
                                global_last_pkt_id
                                - global_first_pkt_id
                                + 1
                            )
                        else:
                            total_stream_packets = 0

                        loss_rate = (
                            estimated_lost_packets
                            / total_stream_packets
                            if total_stream_packets > 0
                            else 0.0
                        )

                        print(
                            "[DCA1000] "
                            f"chunks={total_chunks}, "
                            f"frames={total_saved_frames}, "
                            f"received_packets="
                            f"{total_received_packets}, "
                            f"pending_uint16="
                            f"{pending_uint16.size}, "
                            f"loss="
                            f"{loss_rate * 100:.3f}%"
                        )

        except KeyboardInterrupt:
            print(
                "\n[DCA1000] Manual stop received "
                "by Ctrl+C."
            )

        finally:
            discarded_uint16 = int(
                pending_uint16.size
            )

            if (
                global_first_pkt_id is not None
                and global_last_pkt_id is not None
            ):
                total_stream_packets = (
                    global_last_pkt_id
                    - global_first_pkt_id
                    + 1
                )
            else:
                total_stream_packets = 0

            loss_rate = (
                estimated_lost_packets
                / total_stream_packets
                if total_stream_packets > 0
                else 0.0
            )

            success = total_saved_frames > 0

            if self.capture_complete_callback is not None:
                self.capture_complete_callback(
                    success=success,
                    filename=filename,
                    num_frames=total_saved_frames,
                    packet_count=total_received_packets,
                )

            print(
                "\n========== Infinite Capture Report "
                f"| File: {filename} =========="
            )
            print(
                f"Total chunks:             "
                f"{total_chunks}"
            )
            print(
                f"Total received packets:   "
                f"{total_received_packets}"
            )
            print(
                f"First packet ID:          "
                f"{global_first_pkt_id}"
            )
            print(
                f"Last packet ID:           "
                f"{global_last_pkt_id}"
            )
            print(
                f"Total packets in stream:  "
                f"{total_stream_packets}"
            )
            print(
                f"Estimated lost packets:   "
                f"{estimated_lost_packets}"
            )
            print(
                f"Estimated loss rate:      "
                f"{loss_rate * 100:.3f} %"
            )
            print(
                f"Saved uint16 samples:     "
                f"{total_saved_uint16}"
            )
            print(
                f"Complete saved frames:    "
                f"{total_saved_frames}"
            )
            print(
                f"Discarded tail uint16:    "
                f"{discarded_uint16}"
            )
            print(
                f"Next frame index:         "
                f"{next_frame_idx}"
            )
            print(
                f"Capture success flag:     "
                f"{success}"
            )
            print(
                "================================================"
                "===============\n"
            )

            print(
                f"[DCA1000] File saved to: "
                f"{filename}"
            )

        return {
            "success": bool(success),
            "filename": filename,
            "total_chunks": int(total_chunks),
            "saved_frames": int(total_saved_frames),
            "saved_uint16": int(total_saved_uint16),
            "discarded_uint16": int(
                discarded_uint16
            ),
            "received_packets": int(
                total_received_packets
            ),
            "lost_packets": int(
                estimated_lost_packets
            ),
            "loss_rate": float(loss_rate),
            "next_frame_idx": int(
                next_frame_idx
            ),
        }

    def write_frames_to_file_with_loss_rate(self, 
                                            filename="raw_data.bin", 
                                            numframes=1, 
                                            stop_event=None,
                                            frame_callback=None,
                                            frame_period_s=0.05,
                                            radar_start_time=0,
                                            start_frame_idx=0,):

        numframes = int(numframes)

        if numframes <= 0:
            return self.write_frames_to_file_until_stop(
                filename=filename,
                chunk_frames=100,
                stop_event=stop_event,
                frame_callback=frame_callback,
                frame_period_s=frame_period_s,
                radar_start_time=radar_start_time,
                start_frame_idx=start_frame_idx,
            )

        packetNum = math.ceil(PACKETS_IN_FRAME * numframes)

        recvData = fpga_udp.read_data_udp(
            self.data_socket.fileno(),
            packetNum,
            BYTES_OF_PACKET,
            6
        )

        valid_len = (recvData.size // BYTES_OF_PACKET) * BYTES_OF_PACKET
        if valid_len <= 0:
            raise RuntimeError("[DCA1000] No valid UDP packet received")

        if valid_len != recvData.size:
            print(f"[DCA1000] Warning: truncate incomplete UDP tail: "
                f"{recvData.size} -> {valid_len}")

        recvData = recvData[:valid_len]
        packetNum_real = valid_len // BYTES_OF_PACKET

        print(f"[DCA1000] expected packets: {packetNum}")
        print(f"[DCA1000] received packets: {packetNum_real}")

        recvData = np.reshape(recvData, (packetNum_real, BYTES_OF_PACKET))
        recvQueue = list(map(lambda x: bytes(x), recvData))

        received_packet_ids = []
        for packet in recvQueue:
            pkt_id = struct.unpack('<1L', packet[:4])[0]
            received_packet_ids.append(pkt_id)

        first_pkt_id = min(received_packet_ids)
        last_pkt_id = max(received_packet_ids)
        total_packets_in_sequence = last_pkt_id - first_pkt_id + 1
        received_unique_count = len(set(received_packet_ids))
        lost_count = total_packets_in_sequence - received_unique_count
        loss_rate = lost_count / total_packets_in_sequence if total_packets_in_sequence > 0 else 0.0

        receivedData, firstPacketNum, receivedPacketNum = self.postProcPacket(
            recvQueue,
            packetNum_real
        )

        expected_uint16 = numframes * UINT16_IN_FRAME
        valid_uint16 = min(expected_uint16, receivedData.size)
        databuf = receivedData[:valid_uint16]

        databuf.tofile(filename)

        success = packetNum_real >= packetNum * 0.98

        if self.capture_complete_callback is not None:
            self.capture_complete_callback(
                success=success,
                filename=filename,
                num_frames=numframes,
                packet_count=packetNum_real
            )

        print(f"\n========== Packet Loss Report | File: {filename} ==========")
        print(f"Expected packets:        {packetNum}")
        print(f"Actually received:       {packetNum_real}")
        print(f"First packet ID:         {first_pkt_id}")
        print(f"Last packet ID:          {last_pkt_id}")
        print(f"Total packets in stream: {total_packets_in_sequence}")
        print(f"Received unique packets: {received_unique_count}")
        print(f"Lost packets:            {lost_count}")
        print(f"Packet loss rate:        {loss_rate * 100:.3f} %")
        print(f"Saved uint16 samples:    {valid_uint16}")
        print(f"Capture success flag:    {success}")
        print(f"=============================================================\n")

        print(f"File saved to: {filename}")
    
    def write_udpPakets_to_file(self, filename="upd_data.bin", numPakets=1):
        data, addr = self.data_socket.recvfrom(MAX_PACKET_SIZE)  # MAX_BYTES_PER_PACKET
        data_buf = np.frombuffer(data, dtype=np.uint16)
        for i in range(numPakets - 1):
            print("%d udp received" % (i + 2))
            data_buf = np.append(data_buf, self.data_socket.recvfrom(MAX_PACKET_SIZE))
        data_buf.tofile(filename)

    def write_payloadPakets_to_file(self, filename="pld_data.bin", numPakets=1):
        import time
        start = time.time()
        # packet_num, byte_count, packet_data = self._read_data_packet()
        # data, addr = self.data_socket.recvfrom(MAX_BYTES_PER_PACKET)
        # data_buf = packet_data
        # print("1 packets received")
        # print(packet_num, byte_count)
        for i in range(numPakets):
            # packet_num, byte_count, packet_data = self._read_data_packet()
            data, addr = self.data_socket.recvfrom(MAX_BYTES_PER_PACKET)
            # data_buf = np.append(data_buf,packet_data)
            # print("%d packets received"%(i+2))
            # print(packet_num, byte_count,packet_data.shape)
        end = time.time()
        print("%d packets received" % (i + 1))
        # data, addr = self.data_socket.recvfrom(MAX_BYTES_PER_PACKET)  # MAX_PACKET_SIZE
        packet_num = struct.unpack('<1l', data[:4])[0]
        byte_count = struct.unpack('>Q', b'\x00\x00' + data[4:10][::-1])[0]
        print(packet_num, byte_count)
        print('Running time: %s Seconds' % (end - start), ((byte_count / (end - start)) * 8) / (2 ** 20))
        # data_buf.tofile(filename)

    def sys_alive_check(self):
        # SYSTEM_CONNECT_CMD_CODE
        # 5a a5 09 00 00 00 aa ee
        res = self._send_command(CMD.SYSTEM_CONNECT_CMD_CODE)
        if res:
            res = struct.unpack("<HHHH", res)
            if (res[0] != HEADER_Num or
                    res[1] != struct.unpack("<H", codecs.decode(str(CMD.SYSTEM_CONNECT_CMD_CODE), 'hex'))[0] or
                    res[3] != FOOTER_Num):
                print("receive upd packet error")
                return res
            return STATUS_STR[res[2]]

    def read_fpga_version(self):
        # READ_FPGA_VERSION_CMD_CODE
        # 5a a5 0e 00 00 00 aa ee
        res = self._send_command(CMD.READ_FPGA_VERSION_CMD_CODE)
        if res:
            res = struct.unpack("<HHHH", res)
            if (res[0] != HEADER_Num or
                    res[1] != struct.unpack("<H", codecs.decode(str(CMD.READ_FPGA_VERSION_CMD_CODE), 'hex'))[0] or
                    res[3] != FOOTER_Num):
                print("receive upd packet error")
                return res
            verNum = res[2]
            MajorVersion = verNum & VERSION_BITS_DECODE
            MinorVersion = (verNum >> VERSION_NUM_OF_BITS) & VERSION_BITS_DECODE
            retStr = 'FPGA Version : {}.{} '.format(MajorVersion, MinorVersion)
            if (verNum & PLAYBACK_BIT_DECODE):
                retStr += '[Playback]'
            else:
                retStr += '[Record]'
            return retStr

    def config_fpga(self, filepath):
        # CONFIG_FPGA_GEN_CMD_CODE
        # 5a a5 03 00 06 00 01 02 01 02 03 1e aa ee
        ConfigLogMode = {'raw': '01', 'multi': '02'}
        ConfigLvdsMode = {1: '01', 2: '02'}
        ConfigTransferMode = {'LVDSCapture': '01', 'playback': '02'}
        ConfigCaptureMode = {'SDCardStorage': '01', 'ethernetStream': '02'}
        ConfigFormatMode = {1: '01', 2: '02', 3: '03'}
        ConfigTimer = '1e'  # not support

        with open(filepath, 'r', encoding='utf8')as fp:
            json_data = json.load(fp)
            payload = (ConfigLogMode[json_data['DCA1000Config']['dataLoggingMode']] +
                       ConfigLvdsMode[json_data['DCA1000Config']['lvdsMode']] +
                       ConfigTransferMode[json_data['DCA1000Config']['dataTransferMode']] +
                       ConfigCaptureMode[json_data['DCA1000Config']['dataCaptureMode']] +
                       ConfigFormatMode[json_data['DCA1000Config']['dataFormatMode']] +
                       ConfigTimer)
        packLen = str(codecs.encode(struct.pack('<H', len(codecs.decode(payload, 'hex'))), 'hex'), encoding="utf-8")
        res = self._send_command(CMD.CONFIG_FPGA_GEN_CMD_CODE, packLen, payload)
        if res:
            res = struct.unpack("<HHHH", res)
            if (res[0] != HEADER_Num or
                    res[1] != struct.unpack("<H", codecs.decode(str(CMD.CONFIG_FPGA_GEN_CMD_CODE), 'hex'))[0] or
                    res[3] != FOOTER_Num):
                print("receive upd packet error")
                return res
            return STATUS_STR[res[2]]

    def config_record(self, filepath):
        # CONFIG_PACKET_DATA_CMD_CODE
        # 5a a5 0b 00 06 00 be 05 35 0c 00 00 aa ee
        bytesPerPacket = str(codecs.encode(struct.pack('<H', MAX_BYTES_PER_PACKET), 'hex'), encoding="utf-8")
        reserved = '0000'

        with open(filepath, 'r', encoding='utf8')as fp:
            json_data = json.load(fp)
            recordDelay = str(codecs.encode(struct.pack('<H', int(
                json_data['DCA1000Config']['packetDelay_us'] *
                FPGA_CLK_CONVERSION_FACTOR /
                FPGA_CLK_PERIOD_IN_NANO_SEC
            )), 'hex'), encoding="utf-8")
            payload = (bytesPerPacket + recordDelay + reserved)
        packLen = str(codecs.encode(struct.pack('<H', len(codecs.decode(payload, 'hex'))), 'hex'), encoding="utf-8")
        res = self._send_command(CMD.CONFIG_PACKET_DATA_CMD_CODE, packLen, payload)
        if res:
            res = struct.unpack("<HHHH", res)
            if (res[0] != HEADER_Num or
                    res[1] != struct.unpack("<H", codecs.decode(str(CMD.CONFIG_PACKET_DATA_CMD_CODE), 'hex'))[0] or
                    res[3] != FOOTER_Num):
                print("receive upd packet error")
                return res
            return STATUS_STR[res[2]]

    def _send_command(self, cmd, length='0000', body='', timeout=1):
        """Helper function to send a single commmand to the FPGA

        Args:
            cmd (CMD): Command code to send to the FPGA
            length (str): Length of the body of the command (if any)
            body (str): Body information of the command
            timeout (int): Time in seconds to wait for socket data until timeout

        Returns:
            str: Response message

        """
        # Create timeout exception
        self.config_socket.settimeout(timeout)

        # Create and send message
        resp = ''
        msg = codecs.decode(''.join((CONFIG_HEADER, str(cmd), length, body, CONFIG_FOOTER)), 'hex')
        try:
            self.config_socket.sendto(msg, self.cfg_dest)
            resp, addr = self.config_socket.recvfrom(MAX_PACKET_SIZE)
        except socket.timeout as e:
            print(e)
        return resp

    def _read_data_packet(self):
        """Helper function to read in a single ADC packet via UDP

        Returns:
            int: Current packet number, byte count of data that has 
            already been read, raw ADC data in current packet

        """
        data, addr = self.data_socket.recvfrom(MAX_BYTES_PER_PACKET)  # MAX_PACKET_SIZE
        packet_num = struct.unpack('<1l', data[:4])[0]
        byte_count = struct.unpack('>Q', b'\x00\x00' + data[4:10][::-1])[0]
        packet_data = np.frombuffer(data[10:], dtype=np.uint16)
        # packet_data = data[10:]
        return packet_num, byte_count, packet_data


    def _listen_for_error(self):
        """Helper function to try and read in for an error message from the FPGA

        Returns:
            None

        """
        self.config_socket.settimeout(None)
        msg = self.config_socket.recvfrom(MAX_PACKET_SIZE)
        if msg == b'5aa50a000300aaee':
            print('stopped:', msg)

    def reset_fpga(self):
        res = self._send_command(CMD.RESET_FPGA_CMD_CODE)
        if res:
            res = struct.unpack("<HHHH", res)
            if (res[0] != HEADER_Num or
                    res[1] != struct.unpack("<H", codecs.decode(str(CMD.RESET_FPGA_CMD_CODE), 'hex'))[0] or
                    res[3] != FOOTER_Num):
                print("receive upd packet error")
                return res
            return STATUS_STR[res[2]]

    def reset_radar(self):
        res = self._send_command(CMD.RESET_AR_DEV_CMD_CODE)
        if res:
            res = struct.unpack("<HHHH", res)
            if (res[0] != HEADER_Num or
                    res[1] != struct.unpack("<H", codecs.decode(str(CMD.RESET_AR_DEV_CMD_CODE), 'hex'))[0] or
                    res[3] != FOOTER_Num):
                print("receive upd packet error")
                return res
            return STATUS_STR[res[2]]

    def stream_start(self):
        res = self._send_command(CMD.RECORD_START_CMD_CODE)
        if res:
            res = struct.unpack("<HHHH", res)
            if (res[0] != HEADER_Num or
                    res[1] != struct.unpack("<H", codecs.decode(str(CMD.RECORD_START_CMD_CODE), 'hex'))[0] or
                    res[3] != FOOTER_Num):
                print("receive upd packet error")
                return res
            print("Start streaming")
            return STATUS_STR[res[2]]

    def stream_stop(self):
        """Helper function to send the stop command to the FPGA

        Returns:
            str: Response Message

        """
        res = self._send_command(CMD.RECORD_STOP_CMD_CODE)
        if res:
            res = struct.unpack("<HHHH", res)
            if (res[0] != HEADER_Num or
                    res[1] != struct.unpack("<H", codecs.decode(str(CMD.RECORD_STOP_CMD_CODE), 'hex'))[0] or
                    res[3] != FOOTER_Num):
                print("receive upd packet error")
                return res
            self.data_socket.setblocking(True)
            print("Stop streaming")
            return STATUS_STR[res[2]]

    @staticmethod
    def organize(raw_frame, num_chirps, num_rx, num_samples, Qfirst=True):
        """Reorganizes raw ADC data into a full frame

        Args:
            raw_frame (ndarray): Data to format
            num_chirps: Number of chirps included in the frame
            num_rx: Number of receivers used in the frame
            num_samples: Number of ADC samples included in each chirp
            Qfirst: order of IQ in data file, True for Q first, False for I first
                    Note: by default, mmwave-sdk is Q first while mmwave-studio is I first.
                    Tips: you can check who is first by plotting time domain curves of both I and Q,
                          and observe their phase, Q is delayed by pi/2 phases from I.

        Returns:
            ndarray: Reformatted frame of raw data of shape (num_chirps, num_rx, num_samples)

        """
        ret = np.zeros(len(raw_frame) // 2, dtype=complex)

        # Separate IQ data
        if (Qfirst):
            ret[0::2] = 1j * raw_frame[0::4] + raw_frame[2::4]
            ret[1::2] = 1j * raw_frame[1::4] + raw_frame[3::4]
        else:
            ret[0::2] = raw_frame[0::4] + 1j * raw_frame[2::4]
            ret[1::2] = raw_frame[1::4] + 1j * raw_frame[3::4]
        return ret.reshape((num_chirps, num_rx, num_samples))
