# device={
#     "identifier":'5997',
#     "macAdress": "F2:C3:34:55:0A:DF",
#     "connectionName":"GoPro_Profiler",
#     "SSID":"GP25245997",
#     "password":"MPm-cRX-rZc"
#


class GoProfiler(object):
    """docstring for GoProfiler."""

    def __init__(self):
        super(GoProfiler, self).__init__()
        self.interface="Intel(R) Wi-Fi 6 AX200 160MHz"
        self.identifier ='5870'
        self.timeout = 5
        self.clipDuration = 2                       #Time duration for clips
        self.macAdress = "F2:C3:34:55:0A:DF"
        self.connectionName = "GoPro_Profiler"
        self.SSID ="GP25245997"
        self.password ="MPm-cRX-rZc"
        self.recordingTime = 10
        self.DownloadedFiles=r"C:\Users\Perfilador ResCoast\Desktop\video-test-files"
