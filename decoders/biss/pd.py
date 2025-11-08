import sigrokdecode as srd


class Decoder(srd.Decoder):
    api_version = 3
    id = "biss"
    name = "BISS"
    longname = "Bidirectional/Serial/Synchronous"
    desc = "Fast synchronous serial with differential signaling and line-delay compensation"
    license = "gplv2+"
    inputs = ["logic"]
    outputs = ["biss"]
    tags = ["Embedded/industrial"]
    channels = (
        {"id": "ma", "name": "MA", "desc": "Clock"},
        {"id": "sl", "name": "SL", "desc": "Sensor data"},
    )
    optional_channels = ({"id": "mo", "name": "MO", "desc": "Actuator data"},)
    options = (
        {"id": "d_len", "description": "data length", "default": 38},
        {"id": "crc_len", "description": "crc length", "default": 6},
        {"id": "crc", "description": "CRC polynomial", "default": 0x43},
    )
    annotations = (
        ("bit", "data bit"),  # 0
        ("ack", "Slave acknowledgement"),  # 1
        ("start", "Start bit"),  # 2
        ("cds", "Control communication slave->master"),  # 3
        ("cdm", "Control communication master->slave"),  # 4
        ("linedelay", "Line Delay"),  # 5
        ("processing", "Slave processing time"),  # 6
        ("timeout", "Slave timeout"),  # 7
        ("data", "Data"),  # 8
        ("crc", "CRC"),  # 9
        ("bitcount", "Number of bits in message"),  # 10
    )
    annotation_rows = (
        ("bits", "Bits", (0, 1, 2, 3, 4)),
        ("times", "Times", (5, 6, 7, 10)),
        ("data", "Data", (8, 9)),
    )

    def __init__(self, **kwargs):
        self.state = "IDLE"

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.out_clockrate = self.register(
            srd.OUTPUT_META, meta=(int, "Clockrate", "Master clock rate")
        )

    def reset(self):
        self.samplerate = None

    def decode(self):
        while True:
            self.wait({0: "r"})
            self.wait({0: "r"})  # Wait for the second rising edge
            self.linedelay = self.samplenum
            self.wait({1: "f"})  # Wait for ACK
            self.ack = self.samplenum
            self.ld = self.ack - self.linedelay
            self.put(
                self.linedelay,
                self.ack,
                self.out_ann,
                [5, [f"{self.ld/self.samplerate*1e6:0.2f}us Line Delay", "LD", "L"]],
            )
            self.clk_count = 0
            while True:
                self.clk_count += 1
                self.wait(
                    [{1: "r"}, {0: "e"}]
                )  # Count clock edges while waiting for start bit
                if self.matched[0]:
                    break
            self.sb = self.samplenum
            self.clk_half_period = (self.sb - self.ack) / self.clk_count
            self.cp = int(self.clk_half_period)
            self.put(
                self.sb,
                self.ack,
                self.out_clockrate,
                self.samplerate / self.clk_half_period / 2,
            )
            self.put(self.ack, self.sb, self.out_ann, [1, ["ACK", "A"]])
            n = (self.sb - self.ack) / self.samplerate * 1e6
            self.put(
                self.ack,
                self.sb,
                self.out_ann,
                [6, [f"{n:0.1f}us Processing", f"{n:0.1f}"]],
            )
            #Start bit
            self.wait({0:'l'})
            self.put(
                self.samplenum - self.cp + self.ld,
                self.samplenum + self.cp + self.ld,
                self.out_ann,
                [2, ["START", "ST", "S"]],
            )
            #CDS bit
            ma, sl, _ = self.wait({0:'f'})
            self.put(
                self.samplenum - self.cp + self.ld,
                self.samplenum + self.cp + self.ld,
                self.out_ann,
                [3, [f"CDS: {sl}","CDS", "C"]],
            )
            #Data bits
            self.bitcount = 0
            self.data_word = 0
            while True:
                # FIXME: Use the line delay to sample at the correct times
                ma, sl, _ = self.wait([{0: "f"}, {"skip": 4 * self.cp + 1}])
                if self.matched[1]:
                    break
                self.bitcount += 1
                self.put(
                    self.samplenum - self.cp + self.ld,
                    self.samplenum + self.cp + self.ld,
                    self.out_ann,
                    [0, [f"{sl:1d}"]],
                )
                self.data_word = self.data_word << 1 | sl
            self.timeout = self.samplenum + self.ld - 2*self.cp
            self.put(
                self.sb,
                self.timeout,
                self.out_ann,
                [10, [f"{self.bitcount:2d} bits", f"{self.bitcount:2d}"]],
            )
            self.wait({0: "h", 1: "h"})
            self.idle = self.samplenum
            n = (self.idle - self.timeout) / self.samplerate * 1e6
            self.put(
                self.timeout,
                self.idle,
                self.out_ann,
                [7, [f"{n:0.1f}us Timeout", f"{n:0.1f}"]],
            )
