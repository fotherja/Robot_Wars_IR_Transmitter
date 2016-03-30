/**
 * This CRC table for repeating bytes is take from
 * the cube64 project
 *  http://cia.vc/stats/project/navi-misc/cube64
 */
unsigned char crc_repeating_table[] = {
        	0xFF, // 0x00
		0x14, // 0x01
		0xAC, // 0x02
		0x47, // 0x03
		0x59, // 0x04
		0xB2, // 0x05
		0x0A, // 0x06
		0xE1, // 0x07
		0x36, // 0x08
		0xDD, // 0x09
		0x65, // 0x0A
		0x8E, // 0x0B
		0x90, // 0x0C
		0x7B, // 0x0D
		0xC3, // 0x0E
		0x28, // 0x0F
		0xE8, // 0x10
		0x03, // 0x11
		0xBB, // 0x12
		0x50, // 0x13
		0x4E, // 0x14
		0xA5, // 0x15
		0x1D, // 0x16
		0xF6, // 0x17
		0x21, // 0x18
		0xCA, // 0x19
		0x72, // 0x1A
		0x99, // 0x1B
		0x87, // 0x1C
		0x6C, // 0x1D
		0xD4, // 0x1E
		0x3F, // 0x1F
		0xD1, // 0x20
		0x3A, // 0x21
		0x82, // 0x22
		0x69, // 0x23
		0x77, // 0x24
		0x9C, // 0x25
		0x24, // 0x26
		0xCF, // 0x27
		0x18, // 0x28
		0xF3, // 0x29
		0x4B, // 0x2A
		0xA0, // 0x2B
		0xBE, // 0x2C
		0x55, // 0x2D
		0xED, // 0x2E
		0x06, // 0x2F
		0xC6, // 0x30
		0x2D, // 0x31
		0x95, // 0x32
		0x7E, // 0x33
		0x60, // 0x34
		0x8B, // 0x35
		0x33, // 0x36
		0xD8, // 0x37
		0x0F, // 0x38
		0xE4, // 0x39
		0x5C, // 0x3A
		0xB7, // 0x3B
		0xA9, // 0x3C
		0x42, // 0x3D
		0xFA, // 0x3E
		0x11, // 0x3F
		0xA3, // 0x40
		0x48, // 0x41
		0xF0, // 0x42
		0x1B, // 0x43
		0x05, // 0x44
		0xEE, // 0x45
		0x56, // 0x46
		0xBD, // 0x47
		0x6A, // 0x48
		0x81, // 0x49
		0x39, // 0x4A
		0xD2, // 0x4B
		0xCC, // 0x4C
		0x27, // 0x4D
		0x9F, // 0x4E
		0x74, // 0x4F
		0xB4, // 0x50
		0x5F, // 0x51
		0xE7, // 0x52
		0x0C, // 0x53
		0x12, // 0x54
		0xF9, // 0x55
		0x41, // 0x56
		0xAA, // 0x57
		0x7D, // 0x58
		0x96, // 0x59
		0x2E, // 0x5A
		0xC5, // 0x5B
		0xDB, // 0x5C
		0x30, // 0x5D
		0x88, // 0x5E
		0x63, // 0x5F
		0x8D, // 0x60
		0x66, // 0x61
		0xDE, // 0x62
		0x35, // 0x63
		0x2B, // 0x64
		0xC0, // 0x65
		0x78, // 0x66
		0x93, // 0x67
		0x44, // 0x68
		0xAF, // 0x69
		0x17, // 0x6A
		0xFC, // 0x6B
		0xE2, // 0x6C
		0x09, // 0x6D
		0xB1, // 0x6E
		0x5A, // 0x6F
		0x9A, // 0x70
		0x71, // 0x71
		0xC9, // 0x72
		0x22, // 0x73
		0x3C, // 0x74
		0xD7, // 0x75
		0x6F, // 0x76
		0x84, // 0x77
		0x53, // 0x78
		0xB8, // 0x79
		0x00, // 0x7A
		0xEB, // 0x7B
		0xF5, // 0x7C
		0x1E, // 0x7D
		0xA6, // 0x7E
		0x4D, // 0x7F
		0x47, // 0x80
		0xAC, // 0x81
		0x14, // 0x82
		0xFF, // 0x83
		0xE1, // 0x84
		0x0A, // 0x85
		0xB2, // 0x86
		0x59, // 0x87
		0x8E, // 0x88
		0x65, // 0x89
		0xDD, // 0x8A
		0x36, // 0x8B
		0x28, // 0x8C
		0xC3, // 0x8D
		0x7B, // 0x8E
		0x90, // 0x8F
		0x50, // 0x90
		0xBB, // 0x91
		0x03, // 0x92
		0xE8, // 0x93
		0xF6, // 0x94
		0x1D, // 0x95
		0xA5, // 0x96
		0x4E, // 0x97
		0x99, // 0x98
		0x72, // 0x99
		0xCA, // 0x9A
		0x21, // 0x9B
		0x3F, // 0x9C
		0xD4, // 0x9D
		0x6C, // 0x9E
		0x87, // 0x9F
		0x69, // 0xA0
		0x82, // 0xA1
		0x3A, // 0xA2
		0xD1, // 0xA3
		0xCF, // 0xA4
		0x24, // 0xA5
		0x9C, // 0xA6
		0x77, // 0xA7
		0xA0, // 0xA8
		0x4B, // 0xA9
		0xF3, // 0xAA
		0x18, // 0xAB
		0x06, // 0xAC
		0xED, // 0xAD
		0x55, // 0xAE
		0xBE, // 0xAF
		0x7E, // 0xB0
		0x95, // 0xB1
		0x2D, // 0xB2
		0xC6, // 0xB3
		0xD8, // 0xB4
		0x33, // 0xB5
		0x8B, // 0xB6
		0x60, // 0xB7
		0xB7, // 0xB8
		0x5C, // 0xB9
		0xE4, // 0xBA
		0x0F, // 0xBB
		0x11, // 0xBC
		0xFA, // 0xBD
		0x42, // 0xBE
		0xA9, // 0xBF
		0x1B, // 0xC0
		0xF0, // 0xC1
		0x48, // 0xC2
		0xA3, // 0xC3
		0xBD, // 0xC4
		0x56, // 0xC5
		0xEE, // 0xC6
		0x05, // 0xC7
		0xD2, // 0xC8
		0x39, // 0xC9
		0x81, // 0xCA
		0x6A, // 0xCB
		0x74, // 0xCC
		0x9F, // 0xCD
		0x27, // 0xCE
		0xCC, // 0xCF
		0x0C, // 0xD0
		0xE7, // 0xD1
		0x5F, // 0xD2
		0xB4, // 0xD3
		0xAA, // 0xD4
		0x41, // 0xD5
		0xF9, // 0xD6
		0x12, // 0xD7
		0xC5, // 0xD8
		0x2E, // 0xD9
		0x96, // 0xDA
		0x7D, // 0xDB
		0x63, // 0xDC
		0x88, // 0xDD
		0x30, // 0xDE
		0xDB, // 0xDF
		0x35, // 0xE0
		0xDE, // 0xE1
		0x66, // 0xE2
		0x8D, // 0xE3
		0x93, // 0xE4
		0x78, // 0xE5
		0xC0, // 0xE6
		0x2B, // 0xE7
		0xFC, // 0xE8
		0x17, // 0xE9
		0xAF, // 0xEA
		0x44, // 0xEB
		0x5A, // 0xEC
		0xB1, // 0xED
		0x09, // 0xEE
		0xE2, // 0xEF
		0x22, // 0xF0
		0xC9, // 0xF1
		0x71, // 0xF2
		0x9A, // 0xF3
		0x84, // 0xF4
		0x6F, // 0xF5
		0xD7, // 0xF6
		0x3C, // 0xF7
		0xEB, // 0xF8
		0x00, // 0xF9
		0xB8, // 0xFA
		0x53, // 0xFB
		0x4D, // 0xFC
		0xA6, // 0xFD
		0x1E, // 0xFE
		0xF5 // 0xFF
};