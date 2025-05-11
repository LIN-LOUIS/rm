#ifndef MESSAGE_HPP
#define MESSAGE_HPP

enum MessageType {
    STRING_MSG = 0x0000,
    IMAGE_MSG = 0x1145,
    CAMERA_INFO = 0x1419,
    TRANSFORM = 0x1981,
    TRANSFORM_REQUEST = 0x1982
};

const unsigned short START_SYMBOL = 0x0D00;
const unsigned short END_SYMBOL   = 0x0721;

struct MessageBuffer {
    unsigned short Start;
    unsigned short MessageType;
    unsigned int DataID;
    unsigned int DataTotalLength;
    unsigned int Offset;
    unsigned int DataLength;
    unsigned char Data[10218] = {0};
    unsigned short End;
};

#endif // MESSAGE_HPP
