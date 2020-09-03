import binascii
import sys
import time
import io

crc24table = (
    0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A,
    0x1933EC, 0x9F7F17, 0xA18139, 0x27CDC2, 0x2B5434, 0xAD18CF,
    0x3267D8, 0xB42B23, 0xB8B2D5, 0x3EFE2E, 0xC54E89, 0x430272,
    0x4F9B84, 0xC9D77F, 0x56A868, 0xD0E493, 0xDC7D65, 0x5A319E,
    0x64CFB0, 0xE2834B, 0xEE1ABD, 0x685646, 0xF72951, 0x7165AA,
    0x7DFC5C, 0xFBB0A7, 0x0CD1E9, 0x8A9D12, 0x8604E4, 0x00481F,
    0x9F3708, 0x197BF3, 0x15E205, 0x93AEFE, 0xAD50D0, 0x2B1C2B,
    0x2785DD, 0xA1C926, 0x3EB631, 0xB8FACA, 0xB4633C, 0x322FC7,
    0xC99F60, 0x4FD39B, 0x434A6D, 0xC50696, 0x5A7981, 0xDC357A,
    0xD0AC8C, 0x56E077, 0x681E59, 0xEE52A2, 0xE2CB54, 0x6487AF,
    0xFBF8B8, 0x7DB443, 0x712DB5, 0xF7614E, 0x19A3D2, 0x9FEF29,
    0x9376DF, 0x153A24, 0x8A4533, 0x0C09C8, 0x00903E, 0x86DCC5,
    0xB822EB, 0x3E6E10, 0x32F7E6, 0xB4BB1D, 0x2BC40A, 0xAD88F1,
    0xA11107, 0x275DFC, 0xDCED5B, 0x5AA1A0, 0x563856, 0xD074AD,
    0x4F0BBA, 0xC94741, 0xC5DEB7, 0x43924C, 0x7D6C62, 0xFB2099,
    0xF7B96F, 0x71F594, 0xEE8A83, 0x68C678, 0x645F8E, 0xE21375,
    0x15723B, 0x933EC0, 0x9FA736, 0x19EBCD, 0x8694DA, 0x00D821,
    0x0C41D7, 0x8A0D2C, 0xB4F302, 0x32BFF9, 0x3E260F, 0xB86AF4,
    0x2715E3, 0xA15918, 0xADC0EE, 0x2B8C15, 0xD03CB2, 0x567049,
    0x5AE9BF, 0xDCA544, 0x43DA53, 0xC596A8, 0xC90F5E, 0x4F43A5,
    0x71BD8B, 0xF7F170, 0xFB6886, 0x7D247D, 0xE25B6A, 0x641791,
    0x688E67, 0xEEC29C, 0x3347A4, 0xB50B5F, 0xB992A9, 0x3FDE52,
    0xA0A145, 0x26EDBE, 0x2A7448, 0xAC38B3, 0x92C69D, 0x148A66,
    0x181390, 0x9E5F6B, 0x01207C, 0x876C87, 0x8BF571, 0x0DB98A,
    0xF6092D, 0x7045D6, 0x7CDC20, 0xFA90DB, 0x65EFCC, 0xE3A337,
    0xEF3AC1, 0x69763A, 0x578814, 0xD1C4EF, 0xDD5D19, 0x5B11E2,
    0xC46EF5, 0x42220E, 0x4EBBF8, 0xC8F703, 0x3F964D, 0xB9DAB6,
    0xB54340, 0x330FBB, 0xAC70AC, 0x2A3C57, 0x26A5A1, 0xA0E95A,
    0x9E1774, 0x185B8F, 0x14C279, 0x928E82, 0x0DF195, 0x8BBD6E,
    0x872498, 0x016863, 0xFAD8C4, 0x7C943F, 0x700DC9, 0xF64132,
    0x693E25, 0xEF72DE, 0xE3EB28, 0x65A7D3, 0x5B59FD, 0xDD1506,
    0xD18CF0, 0x57C00B, 0xC8BF1C, 0x4EF3E7, 0x426A11, 0xC426EA,
    0x2AE476, 0xACA88D, 0xA0317B, 0x267D80, 0xB90297, 0x3F4E6C,
    0x33D79A, 0xB59B61, 0x8B654F, 0x0D29B4, 0x01B042, 0x87FCB9,
    0x1883AE, 0x9ECF55, 0x9256A3, 0x141A58, 0xEFAAFF, 0x69E604,
    0x657FF2, 0xE33309, 0x7C4C1E, 0xFA00E5, 0xF69913, 0x70D5E8,
    0x4E2BC6, 0xC8673D, 0xC4FECB, 0x42B230, 0xDDCD27, 0x5B81DC,
    0x57182A, 0xD154D1, 0x26359F, 0xA07964, 0xACE092, 0x2AAC69,
    0xB5D37E, 0x339F85, 0x3F0673, 0xB94A88, 0x87B4A6, 0x01F85D,
    0x0D61AB, 0x8B2D50, 0x145247, 0x921EBC, 0x9E874A, 0x18CBB1,
    0xE37B16, 0x6537ED, 0x69AE1B, 0xEFE2E0, 0x709DF7, 0xF6D10C,
    0xFA48FA, 0x7C0401, 0x42FA2F, 0xC4B6D4, 0xC82F22, 0x4E63D9,
    0xD11CCE, 0x575035, 0x5BC9C3, 0xDD8538)


RTCMV3_PREAMBLE = 0xD3
RTCMV3_MAX_DATA_LENGTH = 1023

# (**********************************************************************
# * Compute the CRC24 checksum using a lookup table method.
# *
# *********************************************************************)


def crc_normal(Message_Buffer):
    crc = 0
    for b in Message_Buffer:
        crc = crc24table[((crc >> 16) ^ b) & 0xFF] ^ (crc << 8)
    return(crc & 0xFFFFFF)


MTI_SUBTYPE_ID = 23
TESEOV_CUSTOM_MESSAGE_NUMBER = 999


class RTCMV3_deframer():

    def __init__(self, read, verbose=True):
        self._read = read
        self.verbose = verbose

    def __iter__(self):
        self._broken = False
        return self

    def __next__(self):
        msg = None
        while msg is None:
            try:
                msg = self._receive()
                if self._broken:
                    raise StopIteration
            except IOError:
                raise StopIteration

        return (msg)

    def _readall(self, size):
        """
        Read until all bytes are collected.

        Parameters
        ----------
        size : int
          Number of bytes to read.
        """
        data = b""
        while len(data) < size:
            d = self._read(size - len(data))
            if not d or self._broken or d == "":
                self._broken = True
                raise StopIteration
            data += d
        return data

    def _receive(self):
        """
        Read and build SBP message.
        """
        # preamble - not readall(1) to allow breaking before messages,
        # empty input
        preamble = self._readall(1)
        if not preamble:
            return None
        elif ord(preamble) != RTCMV3_PREAMBLE:
            if self.verbose:
                print("Host Side Unhandled byte: 0x%02x" % ord(preamble))
            return None
        msg_len_raw = self._readall(2)
        msg_len = (msg_len_raw[0] & 0x3) << 8
        msg_len |= msg_len_raw[1]
        if msg_len > RTCMV3_MAX_DATA_LENGTH and self.verbose:
            print("received too long RTCMV3 msg {}".format(msg_len))
        data = self._readall(msg_len)
        msg_crc = crc_normal(preamble + msg_len_raw + data)
        crc_raw = self._readall(3)
        crc = crc_raw[0] << 16
        crc |= crc_raw[1] << 8
        crc |= crc_raw[2]
        if crc != msg_crc:
            if self.verbose:
                print("crc mismatch: 0x%04X 0x%04X" % (msg_crc, crc))
            return None
        return data


def frame_rtcmv3(in_byte_array):
    msg_len = len(in_byte_array)
    header = bytearray(3)
    header[0] = RTCMV3_PREAMBLE
    header[1] = (msg_len >> 6) & 0x3
    header[2] = msg_len & 0xff
    crc = crc_normal(header + in_byte_array)
    crc_array = bytearray(3)
    crc_array[0] = ((crc >> 16) & 0xff)
    crc_array[1] = ((crc >> 8) & 0xff)
    crc_array[2] = ((crc) & 0xff)
    return header + in_byte_array + crc_array


def get_bitu(buff, bit_pos, length):
    bits = 0
    for bit in range(bit_pos, bit_pos + length):
        if bit >= len(buff) * 8:
            return None
        bits = (bits << 1) + ((buff[bit // 8] >> (7 - bit % 8)) & 1)
    return bits


def get_bits(buff, bit_pos, length):
    unsigned = get_bitu(buff, bit_pos, length)
    val = unsigned
    if (val & (1 << (length - 1))) != 0:  # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << length)   # compute negative value
    return val                          # return positive value as is


def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0:  # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val


def print_imu_temp(read):
    source = RTCMV3_deframer(read)
    for msg in source:
        msg_type = (msg[0] << 4) | ((msg[1] & 0xf0) >> 4)
        if msg_type == 4062:
            sbp_type = ((msg[2] << 8) | (msg[3]))
            if (sbp_type == 2305):
                temp_raw = ((msg[9] << 8) | msg[8])
                print("IMU temperature: {} degrees, {} raw".format(
                    25 + twos_comp(temp_raw, 16)/256.0, temp_raw))


def get_args():
    import argparse

    parser = argparse.ArgumentParser(
        description='Print IMU temperature from a PGM')
    parser.add_argument('filename',
                        help='raw rtcmv3 file to process from PGM board')
    parser.add_argument('--serial', action="store_true",
                        help='interpret file as a serial port. otherwise assume it is afile')
    return parser.parse_args()


if __name__ == "__main__":
    args = get_args()
    if args.serial:
        print("Serial device will be used")
        import serial
        with serial.Serial(args.filename, 460800, timeout=10) as ser:
            print_imu_temp(ser.read)
    else:
        with open(args.filename, 'rb', buffering=32768) as infile:
            print_imu_temp(infile.read)
