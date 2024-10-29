from enum import Enum


class Services(Enum):
    GUEST_INFO = "/guest_info"
    SPEAK = "/speech/speak"
    EXTRACT_DATA = "/extract_data"