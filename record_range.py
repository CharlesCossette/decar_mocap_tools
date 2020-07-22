from data_collector import DataCollector
from pozyx_data_sources import PozyxRangeSource, findPozyxSerial

pozyxs, ids = findPozyxSerial()

pozyx_sources = list()
for pozyx in pozyxs:
    pozyx_sources.append(PozyxRangeSource(pozyx,
                            exclude_ids=ids,
                            allow_self_ranging=False))

dc = DataCollector(*pozyx_sources)
dc.record(300, name = 'position')