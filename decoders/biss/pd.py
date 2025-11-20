import sigrokdecode as srd

def calc_crc(quotient, bits, poly):
    expected = quotient & 0x3F
    quotient -= expected
    for x in range(bits-7, -1, -1):
        if quotient & (0x40 << x):
            quotient ^= poly << x
    res = quotient & 0x3F
    res ^= 0x3F
    return (res, expected)

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
    #optional_channels = ({"id": "mo", "name": "MO", "desc": "Actuator data"},)
    options = (
        {"id": "d_len", "description": "data length", "default": 32},
        {"id": "poly", "description": "CRC polynomial", "default": 0x43},
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
        ("nE", "Error"), #11
        ("nW", "Warning"), #12
        ("crc_error", "CRC Error"), #13
    )
    annotation_rows = (
        ("bits", "Bits", (0, 1, 2, 3, 4)),
        ("times", "Times", (5, 6, 7, 10)),
        ("data", "Data", (8, 9, 13, 11, 12)),
    )
    
    binary = (
        ('rx', 'RX Dump'),
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
        self.out_binary = self.register(srd.OUTPUT_BINARY)
        self.d_len = self.options['d_len']

    def reset(self):
        self.samplerate = None

    def decode(self):
        while True:
            self.wait({0: "r"})
            self.wait({0: "r"})  # Wait for the second rising edge
            ld = self.samplenum
            self.wait({1: "l"})  # Wait for ACK
            ack = self.samplenum
            ld = ack - ld  # line delay
            n = ld/self.samplerate*1e9
            self.put(
                ack-ld,
                ack,
                self.out_ann,
                [5, [f"{n:0.0f}us Line Delay", f"{n:0.0f}ns"]],
            )
            clk_count = 0
            while True:
                clk_count += 1
                self.wait(
                    [{1: "h"}, {0: "e"}]
                )  # Count clock edges while waiting for start bit
                if self.matched[0]:
                    break
            sb = self.samplenum
            clk_half_period = (sb - ack) / clk_count
            cp = int(clk_half_period)
            self.put(
                sb,
                ack,
                self.out_clockrate,
                self.samplerate / clk_half_period / 2,
            )
            self.put(ack, ack+2*cp, self.out_ann, [1, ["ACK", "A"]])
            n = (sb - ack) / self.samplerate * 1e6
            self.put(
                ack,
                sb,
                self.out_ann,
                [6, [f"{n:0.1f}us Setup", f"{n:0.1f}"]],
            )
            # Start bit
            self.wait(
                {0: "l"}
            )  # Rising clock might have been simultaneous with start bit

            # Data bits
            clock_edges = [self.samplenum]
            data_edges = []
            while True:
                ma, sl = self.wait([{0: "r"}, {1: "e"}, {"skip": 4 * cp + 1}])
                if self.matched[1]:  # Data changed, save the previous state
                    data_edges.append((self.samplenum, 1-sl))
                if self.matched[0]: #New clock edge
                    clock_edges.append(self.samplenum + ld)
                if self.matched[2]:  # Clock in timeout, end of data
                    break
            i = 1
            bits = len(clock_edges) * [0]
            for c,v in data_edges:
                while i < len(clock_edges) and clock_edges[i] < c:
                    bits[i] = v
                    i += 1
                if i == len(clock_edges):
                    clock_edges.append(c)
                elif (clock_edges[i] - c) > (c - clock_edges[i-1]):
                    i -= 1
                clock_edges[i] = c
            self.put(
                sb,
                clock_edges[1],
                self.out_ann,
                [2, ["START", "ST", "S"]],
            )
            self.put(
                clock_edges[1],
                clock_edges[2],
                self.out_ann,
                [3, [f'{bits[0]}']],
            )
            data_word = 0
            bitcount = len(clock_edges)-3
            for i in range(2, bitcount+2):
                self.put(clock_edges[i], clock_edges[i+1], self.out_ann, [0, [f'{bits[i]}']])
                data_word = (data_word << 1) | bits[i] 
            if bitcount >= self.d_len + 8:
                self.put(clock_edges[2], clock_edges[2+self.d_len], self.out_ann, [8, [f'{data_word>>(bitcount-self.d_len):0{(self.d_len+3)//4}X}']])
                if not bits[self.d_len+2]:
                    self.put(clock_edges[2+self.d_len], clock_edges[3+self.d_len], self.out_ann, [11, ['Error', 'Err', 'E']])
                if not bits[self.d_len+3]:
                    self.put(clock_edges[3+self.d_len], clock_edges[4+self.d_len], self.out_ann, [12, ['Warning', 'Warn', 'W']])
                expected, crc = calc_crc(data_word>>(bitcount-self.d_len-8), self.d_len+8, self.options['poly'])
                self.put(clock_edges[4+self.d_len], clock_edges[10+self.d_len], self.out_ann, [9 if expected==crc else 13, [f'{expected:02X}']])
            self.put(clock_edges[2], clock_edges[-1], self.out_binary, [0, data_word.to_bytes(-(-bitcount//8), byteorder='big')])
            timeout = clock_edges[-1]
            self.put(
                clock_edges[2],
                clock_edges[-1],
                self.out_ann,
                [10, [f"{bitcount:2d} bits", f"{bitcount:2d}"]],
            )
            self.wait({0: "h", 1: "h"})
            idle = self.samplenum
            n = (idle - timeout) / self.samplerate * 1e6
            self.put(
                timeout,
                idle,
                self.out_ann,
                [7, [f"{n:0.1f}us Timeout", f"{n:0.1f}"]],
            )
