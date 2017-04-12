Header header
float64         number_of_lines  #adaptiveness of the filter! number of lanes that we were looking for
float64         goodLines        #how many good lines are currently found
float64         width            #full road width, calculated with valid line offsets
float64         naive_width      #full road width, calculated with all line offsets (even invalid)
#int32           oneway           #MAKE SENSE? nope, moved to LayoutComponent_RoadState as class parameter
int64           way_id           #FOR COMPATIBILITY WITH roadStateComponent FAKE
msg_lineInfo[]  lines            #details of each line
