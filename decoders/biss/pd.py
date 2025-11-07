import sigrokdecode as srd

class Decoder(srd.Decoder):
    api_version = 3
    id = 'biss'
    name = 'BISS'
    longname = 'Bidirectional/Serial/Synchronous'
    desc = 'Fast synchronous serial with differential signaling and line-delay compensation'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = ['biss']
    channels = (
            {'id':'ma', 'name':'MA', 'desc':'Clock'},
            {'id':'sl', 'name':'SL', 'desc':'Sensor data'},
        )
    optional_channels = (
            {'id':'mo', 'name':'MO', 'desc':'Actuator data'},
        )
    options = (
            {'id':'d_len','description':'data length', 'default':38},
            {'id':'crc_len','description':'crc length', 'default':6},
            {'id':'crc','description':'CRC polynomial', 'default':0x43},
        )
    annotations = (
            ('bit', 'data bit'), #0
            ('ack', 'Slave acknowledgement'), #1
            ('start', 'Start bit'), #2
            ('cds', 'Control communication slave->master'), #3
            ('cdm', 'Control communication master->slave'), #4
            ('linedelay', 'Line Delay'), #5
            ('processing', 'Slave processing time'), #6
            ('timeout', 'Slave timeout'), #7
            ('data', 'Data'), #8
            ('crc', 'CRC'), #9
        )
    annotation_rows = (
            ('bits', 'Bits', (0,1,2,3,4)),
            ('times', 'Times', (5,6,7)),
            ('data', 'Data', (8,9)),
        )
    binary = (
            ('data', 'Data')
        )

    def __init__(self, **kwargs):
        self.state = 'IDLE'

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.out_clockrate = self.register(srd.OUTPUT_META,
                meta=(int, 'Clockrate', 'Master clock rate'))

    def decode(self):
        while True:
            self.wait({0:'r'})
            self.wait({0:'r'}) #Wait for the second rising edge
            self.linedelay = self.samplenum
            self.wait({1:'f'}) #Wait for ACK
            self.ack = self.samplenum
            self.ld = self.ack - self.linedelay
            self.put(self.linedelay, self,ack, self.out_ann, [5, ['LineDelay','LD','L']], self.ld/self.samplerate)
            self.clk_count = 0
            while True:
                self.clk_count += 1
                self.wait({1:'r'}, {0:'e'}) #Count clock edges while waiting for start bit
                if self.matched[0]:
                    break
            self.sb = self.samplenum
            self.clk_half_period = (self.sb-self.ack)/self.clk_count
            self.cp = int(self.clk_half_period)
            self.put(self.sb, self.ack, self.out_clockrate, self.samplerate/self.clk_half_period/2)
            self.put(self.ack, self,sb, self.out_ann, [1, ['ACK','A']])
            self.put(self.ack, self,sb, self.out_ann, [6, ['Processing','Pt']])
            while True:
                #FIXME: Use the line delay
                ma, sl = self.wait({0:f}, {'skip':3*self.cp+1})
                if self.matched[1]:
                    break
                self.put(self.samplenum-self.cp, self.samplenum+self.cp, self.out_ann, [0, 'Bit', 'B'])
            self.timeout = self.samplenum - 2*self.cp
            self.wait({0:'h', 1:'h'})
            self.idle = self.samplenum
            self.put(self.timeout, self.idle, self.out_ann, [7, 'Timeout','T'])
              
              
              
              



    



