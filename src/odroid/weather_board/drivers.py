from __future__ import absolute_import, division, print_function

import time

import smbus

to_s16 = lambda x: (x + 2 ** 15) % 2 ** 16 - 2 ** 15
to_u16 = lambda x: x % 2 ** 16

BME280_I2C_ADDR = 0x76
BME280_CHIP_ID = 0x60

bme280_power_mode = {"BME280_SLEEP_MODE": 0x00, "BME280_FORCED_MODE": 0x01, "BME280_NORMAL_MODE": 0x03,
                     "BME280_SOFT_RESET_CODE": 0xB6, }

bme280_regs = {"BME280_CHIP_ID_REG": 0xD0, "BME280_RST_REG": 0xE0, "BME280_STAT_REG": 0xF3,
               "BME280_CTRL_MEAS_REG": 0xF4, "BME280_CTRL_HUMIDITY_REG": 0xF2, "BME280_CONFIG_REG": 0xF5,
               "BME280_PRESSURE_MSB_REG": 0xF7, "BME280_PRESSURE_LSB_REG": 0xF8, "BME280_PRESSURE_XLSB_REG": 0xF9,
               "BME280_TEMPERATURE_MSB_REG": 0xFA, "BME280_TEMPERATURE_LSB_REG": 0xFB,
               "BME280_TEMPERATURE_XLSB_REG": 0xFC, "BME280_HUMIDITY_MSB_REG": 0xFD, "BME280_HUMIDITY_LSB_REG": 0xFE, }

bme280_cali_regs = {"BME280_TEMPERATURE_CALIB_DIG_T1_LSB_REG": 0x88, "BME280_TEMPERATURE_CALIB_DIG_T1_MSB_REG": 0x89,
                    "BME280_TEMPERATURE_CALIB_DIG_T2_LSB_REG": 0x8A, "BME280_TEMPERATURE_CALIB_DIG_T2_MSB_REG": 0x8B,
                    "BME280_TEMPERATURE_CALIB_DIG_T3_LSB_REG": 0x8C, "BME280_TEMPERATURE_CALIB_DIG_T3_MSB_REG": 0x8D,
                    "BME280_PRESSURE_CALIB_DIG_P1_LSB_REG": 0x8E, "BME280_PRESSURE_CALIB_DIG_P1_MSB_REG": 0x8F,
                    "BME280_PRESSURE_CALIB_DIG_P2_LSB_REG": 0x90, "BME280_PRESSURE_CALIB_DIG_P2_MSB_REG": 0x91,
                    "BME280_PRESSURE_CALIB_DIG_P3_LSB_REG": 0x92, "BME280_PRESSURE_CALIB_DIG_P3_MSB_REG": 0x93,
                    "BME280_PRESSURE_CALIB_DIG_P4_LSB_REG": 0x94, "BME280_PRESSURE_CALIB_DIG_P4_MSB_REG": 0x95,
                    "BME280_PRESSURE_CALIB_DIG_P5_LSB_REG": 0x96, "BME280_PRESSURE_CALIB_DIG_P5_MSB_REG": 0x97,
                    "BME280_PRESSURE_CALIB_DIG_P6_LSB_REG": 0x98, "BME280_PRESSURE_CALIB_DIG_P6_MSB_REG": 0x99,
                    "BME280_PRESSURE_CALIB_DIG_P7_LSB_REG": 0x9A, "BME280_PRESSURE_CALIB_DIG_P7_MSB_REG": 0x9B,
                    "BME280_PRESSURE_CALIB_DIG_P8_LSB_REG": 0x9C, "BME280_PRESSURE_CALIB_DIG_P8_MSB_REG": 0x9D,
                    "BME280_PRESSURE_CALIB_DIG_P9_LSB_REG": 0x9E, "BME280_PRESSURE_CALIB_DIG_P9_MSB_REG": 0x9F,
                    "BME280_HUMIDITY_CALIB_DIG_H1_REG": 0xA1, "BME280_HUMIDITY_CALIB_DIG_H2_LSB_REG": 0xE1,
                    "BME280_HUMIDITY_CALIB_DIG_H2_MSB_REG": 0xE2, "BME280_HUMIDITY_CALIB_DIG_H3_REG": 0xE3,
                    "BME280_HUMIDITY_CALIB_DIG_H4_MSB_REG": 0xE4, "BME280_HUMIDITY_CALIB_DIG_H4_LSB_REG": 0xE5,
                    "BME280_HUMIDITY_CALIB_DIG_H5_MSB_REG": 0xE6, "BME280_HUMIDITY_CALIB_DIG_H6_REG": 0xE7, }

SI1132_I2C_ADDR = 0x60
SI1132_PARTID = 0x32

si1132_cmds = {'Si1132_PARAM_QUERY': 0x80, 'Si1132_PARAM_SET': 0xA0, 'Si1132_NOP': 0x00, 'Si1132_RESET': 0x01,
               'Si1132_BUSADDR': 0x02, 'Si1132_GET_CAL': 0x12, 'Si1132_ALS_FORCE': 0x06, 'Si1132_ALS_PAUSE': 0x0A,
               'Si1132_ALS_AUTO': 0x0E, }

si1132_params = {'Si1132_PARAM_I2CADDR': 0x00, 'Si1132_PARAM_CHLIST': 0x01, 'Si1132_PARAM_CHLIST_ENUV': 0x80,
                 'Si1132_PARAM_CHLIST_ENAUX': 0x40, 'Si1132_PARAM_CHLIST_ENALSIR': 0x20,
                 'Si1132_PARAM_CHLIST_ENALSVIS': 0x10, 'Si1132_PARAM_ALSENCODING': 0x06,
                 'Si1132_PARAM_ALSIRADCMUX': 0x0E, 'Si1132_PARAM_AUXADCMUX': 0x0F,
                 'Si1132_PARAM_ALSVISADCCOUNTER': 0x10, 'Si1132_PARAM_ALSVISADCGAIN': 0x11,
                 'Si1132_PARAM_ALSVISADCMISC': 0x12, 'Si1132_PARAM_ALSVISADCMISC_VISRANGE': 0x20,
                 'Si1132_PARAM_ALSIRADCCOUNTER': 0x1D, 'Si1132_PARAM_ALSIRADCGAIN': 0x1E,
                 'Si1132_PARAM_ALSIRADCMISC': 0x1F, 'Si1132_PARAM_ALSIRADCMISC_RANGE': 0x20,
                 'Si1132_PARAM_ADCCOUNTER_511CLK': 0x70, 'Si1132_PARAM_ADCMUX_SMALLIR': 0x00,
                 'Si1132_PARAM_ADCMUX_LARGEIR': 0x03, }

si1132_regs = {'Si1132_REG_PARTID': 0x00, 'Si1132_REG_REVID': 0x01, 'Si1132_REG_SEQID': 0x02, 'Si1132_REG_INTCFG': 0x03,
               'Si1132_REG_INTCFG_INTOE': 0x01, 'Si1132_REG_IRQEN': 0x04, 'Si1132_REG_IRQEN_ALSEVERYSAMPLE': 0x01,
               'Si1132_REG_IRQMODE1': 0x05, 'Si1132_REG_IRQMODE2': 0x06, 'Si1132_REG_HWKEY': 0x07,
               'Si1132_REG_MEASRATE0': 0x08, 'Si1132_REG_MEASRATE1': 0x09, 'Si1132_REG_UCOEF0': 0x13,
               'Si1132_REG_UCOEF1': 0x14, 'Si1132_REG_UCOEF2': 0x15, 'Si1132_REG_UCOEF3': 0x16,
               'Si1132_REG_PARAMWR': 0x17, 'Si1132_REG_COMMAND': 0x18, 'Si1132_REG_RESPONSE': 0x20,
               'Si1132_REG_IRQSTAT': 0x21, 'Si1132_REG_ALSVISDATA0': 0x22, 'Si1132_REG_ALSVISDATA1': 0x23,
               'Si1132_REG_ALSIRDATA0': 0x24, 'Si1132_REG_ALSIRDATA1': 0x25, 'Si1132_REG_UVINDEX0': 0x2C,
               'Si1132_REG_UVINDEX1': 0x2D, 'Si1132_REG_PARAMRD': 0x2E, 'Si1132_REG_CHIPSTAT': 0x30, }


class BME280:
    param = {}

    def __init__(self, i2c_dev, p_mode, h_samp, p_samp, t_samp):
        self.bus = smbus.SMBus(int(i2c_dev.split('-')[-1]))
        self.chip_id = self.read(bme280_regs['BME280_CHIP_ID_REG'], 1)[0]
        if self.chip_id != BME280_CHIP_ID:
            print("Wrong Device ID [", hex(self.chip_id), "]")
            exit()

        self.config_reg = 0
        self.ctrl_meas_reg = 0
        self.ctrl_hum_reg = 0
        self.oversamp_humidity = 0
        self.oversamp_pressure = 0

        self.get_cal_param()
        self.set_power_mode(p_mode)
        self.set_oversamp_humidity(h_samp)
        self.set_oversamp_pressure(p_samp)
        self.set_oversamp_temperature(t_samp)
        time.sleep(0.01)
        self.t_fine = 0.0

    def read(self, reg, cnt):
        val = [0 for x in range(cnt)]
        for i in range(cnt):
            val[i] = self.bus.read_byte_data(BME280_I2C_ADDR, reg + i)
        return val

    def write(self, reg, val):
        for i in range(len(val)):
            self.bus.write_byte_data(BME280_I2C_ADDR, reg + i, val[i])

    def get_cal_param(self):
        params = self.read(bme280_cali_regs['BME280_TEMPERATURE_CALIB_DIG_T1_LSB_REG'], 26)
        self.param['dig_T1'] = to_u16((params[1] << 8) | params[0])
        self.param['dig_T2'] = to_s16((params[3] << 8) | params[2])
        self.param['dig_T3'] = to_s16((params[5] << 8) | params[4])
        self.param['dig_P1'] = to_u16((params[7] << 8) | params[6])
        self.param['dig_P2'] = to_s16((params[9] << 8) | params[8])
        self.param['dig_P3'] = to_s16((params[11] << 8) | params[10])
        self.param['dig_P4'] = to_s16((params[13] << 8) | params[12])
        self.param['dig_P5'] = to_s16((params[15] << 8) | params[14])
        self.param['dig_P6'] = to_s16((params[17] << 8) | params[16])
        self.param['dig_P7'] = to_s16((params[19] << 8) | params[18])
        self.param['dig_P8'] = to_s16((params[21] << 8) | params[20])
        self.param['dig_P9'] = to_s16((params[23] << 8) | params[22])
        self.param['dig_H1'] = params[25]

        params = self.read(bme280_cali_regs['BME280_HUMIDITY_CALIB_DIG_H2_LSB_REG'], 7)
        self.param['dig_H2'] = to_s16((params[1] << 8) | params[0])
        self.param['dig_H3'] = params[2]
        self.param['dig_H4'] = to_s16((params[3] << 4) | (params[4] & 0x0f))
        self.param['dig_H5'] = to_s16((params[5] << 4) | (params[4] >> 4))
        self.param['dig_H6'] = params[6]

    def get_power_mode(self):
        return self.read(bme280_regs['BME280_CTRL_MEAS_REG'], 1)[0] & 0x03

    def set_power_mode(self, p_mode):
        if p_mode <= bme280_power_mode['BME280_NORMAL_MODE']:
            v_mode = (self.ctrl_meas_reg & ~0x03) | (p_mode & 0x03)

            if self.get_power_mode() != bme280_power_mode['BME280_SLEEP_MODE']:
                self.soft_rst()
                time.sleep(0.003)
                self.write(bme280_regs['BME280_CONFIG_REG'], [self.config_reg])
                self.write(bme280_regs['BME280_CTRL_HUMIDITY_REG'], [self.ctrl_hum_reg])
                self.write(bme280_regs['BME280_CTRL_MEAS_REG'], [v_mode])
            else:
                self.write(bme280_regs['BME280_CTRL_MEAS_REG'], [v_mode])

            self.ctrl_meas_reg = self.read(bme280_regs['BME280_CTRL_MEAS_REG'], 1)[0]
            self.ctrl_hum_reg = self.read(bme280_regs['BME280_CTRL_HUMIDITY_REG'], 1)[0]
            self.config_reg = self.read(bme280_regs['BME280_CONFIG_REG'], 1)[0]
        else:
            print("Wrong Power Mode [", hex(p_mode), "]")
            exit()

    def soft_rst(self):
        self.write(bme280_regs['BME280_RST_REG'], [bme280_power_mode['BME280_SOFT_RESET_CODE']])

    def set_oversamp_humidity(self, sampling):
        v_data = (self.ctrl_hum_reg & ~0x07) | (sampling & 0x07)

        if self.get_power_mode() != bme280_power_mode['BME280_SLEEP_MODE']:
            self.soft_rst()
            time.sleep(0.003)
            self.write(bme280_regs['BME280_CONFIG_REG'], [self.config_reg])
            self.write(bme280_regs['BME280_CTRL_HUMIDITY_REG'], [v_data])
            self.write(bme280_regs['BME280_CTRL_MEAS_REG'], [self.ctrl_meas_reg])
        else:
            self.write(bme280_regs['BME280_CTRL_HUMIDITY_REG'], [v_data])
            self.write(bme280_regs['BME280_CTRL_MEAS_REG'], [self.ctrl_hum_reg])

        self.oversamp_humidity = sampling
        self.ctrl_meas_reg = self.read(bme280_regs['BME280_CTRL_MEAS_REG'], 1)[0]
        self.ctrl_hum_reg = self.read(bme280_regs['BME280_CTRL_HUMIDITY_REG'], 1)[0]
        self.config_reg = self.read(bme280_regs['BME280_CONFIG_REG'], 1)[0]

    def set_oversamp_pressure(self, sampling):
        v_data = (self.ctrl_meas_reg & ~0x1c) | ((sampling << 2) & 0x1c)

        if self.get_power_mode != bme280_power_mode['BME280_SLEEP_MODE']:
            self.soft_rst()
            time.sleep(0.003)
            self.write(bme280_regs['BME280_CONFIG_REG'], [self.config_reg])
            self.write(bme280_regs['BME280_CTRL_HUMIDITY_REG'], [self.ctrl_hum_reg])
            self.write(bme280_regs['BME280_CTRL_MEAS_REG'], [v_data])
        else:
            self.write(bme280_regs['BME280_CTRL_MEAS_REG'], [v_data])

        self.oversamp_pressure = sampling
        self.ctrl_meas_reg = self.read(bme280_regs['BME280_CTRL_MEAS_REG'], 1)[0]
        self.ctrl_hum_reg = self.read(bme280_regs['BME280_CTRL_HUMIDITY_REG'], 1)[0]
        self.config_reg = self.read(bme280_regs['BME280_CONFIG_REG'], 1)[0]

    def set_oversamp_temperature(self, sampling):
        v_data = (self.ctrl_meas_reg & ~0xe0) | ((sampling << 5) & 0xe0)

        if self.get_power_mode != bme280_power_mode['BME280_SLEEP_MODE']:
            self.soft_rst()
            time.sleep(0.003)
            self.write(bme280_regs['BME280_CONFIG_REG'], [self.config_reg])
            self.write(bme280_regs['BME280_CTRL_HUMIDITY_REG'], [self.ctrl_hum_reg])
            self.write(bme280_regs['BME280_CTRL_MEAS_REG'], [v_data])
        else:
            self.write(bme280_regs['BME280_CTRL_MEAS_REG'], [v_data])

        self.oversamp_temperature = sampling
        self.ctrl_meas_reg = self.read(bme280_regs['BME280_CTRL_MEAS_REG'], 1)[0]
        self.ctrl_hum_reg = self.read(bme280_regs['BME280_CTRL_HUMIDITY_REG'], 1)[0]
        self.config_reg = self.read(bme280_regs['BME280_CONFIG_REG'], 1)[0]

    def read_humidity(self):
        vals = self.read(bme280_regs['BME280_PRESSURE_MSB_REG'], 8)
        raw_val = float((vals[6] << 8) | vals[7])
        h = float(self.t_fine) - 76800.0
        h = (raw_val - (float(self.param['dig_H4']) * 64.0 + float(self.param['dig_H5']) / 16384.0 * h)) * (
                float(self.param['dig_H2']) / 65536.0 * (1.0 + float(self.param['dig_H6']) / 67108864.0 * h * (
                1.0 + float(self.param['dig_H3']) / 67108864.0 * h)))
        h = h * (1.0 - float(self.param['dig_H1']) * h / 524288.0)
        if h > 100:
            h = 100
        elif h < 0:
            h = 0
        return h

    def read_pressure(self):
        vals = self.read(bme280_regs['BME280_PRESSURE_MSB_REG'], 8)
        raw_val = float((vals[0] << 12) | (vals[1] << 4) | (vals[2] >> 4))

        var1 = float(self.t_fine) / 2.0 - 64000.0
        var2 = var1 * var1 * float(self.param['dig_P6']) / 32768.0
        var2 = var2 + var1 * float(self.param['dig_P5']) * 2.0
        var2 = var2 / 4.0 + float(self.param['dig_P4']) * 65536.0
        var1 = (float(self.param['dig_P3']) * var1 * var1 / 524288.0 + float(self.param['dig_P2']) * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * float(self.param['dig_P1'])
        if var1 == 0:
            return 0
        p = 1048576.0 - raw_val
        p = ((p - var2 / 4096.0) * 6250.0) / var1
        var1 = float(self.param['dig_P9']) * p * p / 2147483648.0
        var2 = p * float(self.param['dig_P8']) / 32768.0
        p = p + (var1 + var2 + float(self.param['dig_P7'])) / 16.0
        return p

    def read_temperature(self):
        vals = self.read(bme280_regs['BME280_PRESSURE_MSB_REG'], 8)
        raw_val = float((vals[3] << 12) | (vals[4] << 4) | (vals[5] >> 4))

        var1 = (raw_val / 16384.0 - float(self.param['dig_T1']) / 1024.0) * float(self.param['dig_T2'])
        var2 = ((raw_val / 131072.0 - float(self.param['dig_T1']) / 8192.0) * (
                raw_val / 131072.0 - float(self.param['dig_T1']) / 8192.0)) * float(self.param['dig_T3'])
        self.t_fine = int(var1 + var2)
        temp = (var1 + var2) / 5120.0
        return temp


class SI1132:
    def __init__(self, i2c_dev):
        self.bus = smbus.SMBus(int(i2c_dev.split('-')[-1]))
        self.id = self.read_reg8(0x00)
        if self.id != SI1132_PARTID:
            print("Wrong Device ID [", hex(self.id), "]")
            exit()

        self.reset()

        self.write_reg8(si1132_regs['Si1132_REG_UCOEF0'], 0x7B)
        self.write_reg8(si1132_regs['Si1132_REG_UCOEF1'], 0x6B)
        self.write_reg8(si1132_regs['Si1132_REG_UCOEF2'], 0x01)
        self.write_reg8(si1132_regs['Si1132_REG_UCOEF3'], 0x00)

        param = si1132_params['Si1132_PARAM_CHLIST_ENUV'] | si1132_params['Si1132_PARAM_CHLIST_ENALSIR'] | \
                si1132_params[
                    'Si1132_PARAM_CHLIST_ENALSVIS']
        self.write_param(si1132_params['Si1132_PARAM_CHLIST'], param)

        self.write_reg8(si1132_regs['Si1132_REG_INTCFG'], si1132_regs['Si1132_REG_INTCFG_INTOE'])
        self.write_reg8(si1132_regs['Si1132_REG_IRQEN'], si1132_regs['Si1132_REG_IRQEN_ALSEVERYSAMPLE'])

        self.write_param(si1132_params['Si1132_PARAM_ALSIRADCMUX'], si1132_params['Si1132_PARAM_ADCMUX_SMALLIR'])
        time.sleep(0.01)
        self.write_param(si1132_params['Si1132_PARAM_ALSIRADCGAIN'], 0)
        time.sleep(0.01)
        self.write_param(si1132_params['Si1132_PARAM_ALSIRADCCOUNTER'], si1132_params['Si1132_PARAM_ADCCOUNTER_511CLK'])
        self.write_param(si1132_params['Si1132_PARAM_ALSIRADCMISC'], si1132_params['Si1132_PARAM_ALSIRADCMISC_RANGE'])
        time.sleep(0.01)
        self.write_param(si1132_params['Si1132_PARAM_ALSIRADCGAIN'], 0)
        time.sleep(0.01)
        self.write_param(si1132_params['Si1132_PARAM_ALSIRADCCOUNTER'], si1132_params['Si1132_PARAM_ADCCOUNTER_511CLK'])
        self.write_param(si1132_params['Si1132_PARAM_ALSVISADCMISC'],
                         si1132_params['Si1132_PARAM_ALSVISADCMISC_VISRANGE'])
        time.sleep(0.01)
        self.write_reg8(si1132_regs['Si1132_REG_MEASRATE0'], 0xFF)
        self.write_reg8(si1132_regs['Si1132_REG_COMMAND'], si1132_cmds['Si1132_ALS_AUTO'])

    def reset(self):
        self.write_reg8(si1132_regs['Si1132_REG_MEASRATE0'], 0)
        self.write_reg8(si1132_regs['Si1132_REG_MEASRATE1'], 0)
        self.write_reg8(si1132_regs['Si1132_REG_IRQEN'], 0)
        self.write_reg8(si1132_regs['Si1132_REG_IRQMODE1'], 0)
        self.write_reg8(si1132_regs['Si1132_REG_IRQMODE2'], 0)
        self.write_reg8(si1132_regs['Si1132_REG_INTCFG'], 0)
        self.write_reg8(si1132_regs['Si1132_REG_IRQSTAT'], 0xFF)
        self.write_reg8(si1132_regs['Si1132_REG_COMMAND'], si1132_cmds['Si1132_RESET'])
        time.sleep(0.01)
        self.write_reg8(si1132_regs['Si1132_REG_HWKEY'], 0x17)
        time.sleep(0.01)

    def read_reg8(self, reg):
        return self.bus.read_byte_data(SI1132_I2C_ADDR, reg)

    def read_reg16(self, reg):
        return self.bus.read_word_data(SI1132_I2C_ADDR, reg)

    def write_reg8(self, reg, val):
        self.bus.write_byte_data(SI1132_I2C_ADDR, reg, val)

    def write_reg16(self, reg, val):
        self.bus.write_word_data(SI1132_I2C_ADDR, reg, val)

    def write_param(self, param, val):
        self.write_reg8(si1132_regs['Si1132_REG_PARAMWR'], val)
        self.write_reg8(si1132_regs['Si1132_REG_COMMAND'], param | si1132_cmds['Si1132_PARAM_SET'])

    def read_visible(self):
        time.sleep(0.01)
        return ((self.read_reg16(0x22) - 256) / 0.282) * 14.5

    def read_ir(self):
        time.sleep(0.01)
        return ((self.read_reg16(0x24) - 250) / 2.44) * 14.5

    def read_uv(self):
        time.sleep(0.01)
        return self.read_reg16(0x2c)
