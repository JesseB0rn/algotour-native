from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Optional as _Optional

DESCRIPTOR: _descriptor.FileDescriptor

class RouteRequestRPC(_message.Message):
    __slots__ = ("startLon", "startLat", "endLon", "endLat")
    STARTLON_FIELD_NUMBER: _ClassVar[int]
    STARTLAT_FIELD_NUMBER: _ClassVar[int]
    ENDLON_FIELD_NUMBER: _ClassVar[int]
    ENDLAT_FIELD_NUMBER: _ClassVar[int]
    startLon: float
    startLat: float
    endLon: float
    endLat: float
    def __init__(self, startLon: _Optional[float] = ..., startLat: _Optional[float] = ..., endLon: _Optional[float] = ..., endLat: _Optional[float] = ...) -> None: ...

class RouteResponse(_message.Message):
    __slots__ = ("pathFile",)
    PATHFILE_FIELD_NUMBER: _ClassVar[int]
    pathFile: str
    def __init__(self, pathFile: _Optional[str] = ...) -> None: ...
