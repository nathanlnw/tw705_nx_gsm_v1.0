Import('RTT_ROOT')
Import('rtconfig')
from building import *

cwd     = os.path.join(str(Dir('#')), 'drivers','drivers\include') 
src	= Glob('*.c')
src =src+ Glob('CAN/*.c')+Glob('dataflash/*.c')+Glob('dataflash/SST25/*.c')+Glob('gps/*.c')+Glob('gsm/*.c')+Glob('HMI/*.c')+Glob('ICcard/*.c')
src =src+ Glob('lcd/*.c')+Glob('printer/*.c')+Glob('protocol_808/*.c')+Glob('rtc/*.c')+Glob('TFcard/*.c')+Glob('U4_485/*.c')


CPPPATH = [cwd +'/CAN',
          cwd +'/dataflash',
          cwd +'/dataflash/SST25',
          cwd +'/gps',
          cwd +'/gsm',
          cwd +'/HMI',
          cwd +'/ICcard',
          cwd +'/lcd',
          cwd +'/printer',
          cwd +'/protocol_808',
          cwd +'/rtc',
          cwd +'/TFcard',
          cwd +'/U4_485',
          GetCurrentDir()
         ]

group = DefineGroup('Drivers', src, depend = [''], CPPPATH = CPPPATH)

Return('group')
