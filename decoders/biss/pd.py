import sigrokdecode as srd

class Decoder(srd.Decoder):
    api_version = 2
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
            ('bit', 'data bit'),
            ('ack', 'Slave acknowledgement'),
            ('start', 'Start bit'),
            ('cds', 'Control communication slave->master'),
            ('cdm', 'Control communication master->slave'),
            ('linedelay', 'Line Delay'),
            ('processing', 'Slave processing time'),
            ('timeout', 'Slave timeout'),
            ('data', 'Data'),
            ('crc', 'CRC'),
        )
    annotation_rows = (
            ('bits', 'Bits', (0,1,2,3,4)),
            ('times', 'Times', (5,6,7)),
            ('data', 'Data', (8,9)),
        )

    def __init__(self, **kwargs):
        self.state = 'FIND LATCH'

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def decode(self, ss, es, data):
        for self.samplenum, (ma, sl) in data:
            #decode the samples



    



