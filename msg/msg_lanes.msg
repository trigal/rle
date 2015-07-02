int32           number_of_lanes  #adaptiveness of the filter! number of lanes that we were looking for
int32           goodLines        #how many good lines are currently found
float64         width            #full road width, calculated with lane offsets
int32           oneway           #MAKE SENSE?
int64           way_id           #FOR COMPATIBILITY WITH roadStateComponent FAKE
msg_lineInfo[]  lines            #details of each line
