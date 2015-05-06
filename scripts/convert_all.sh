RTK_string=`echo $1 | sed "s#RLE#RTK#g"`
LIBVISO_string=`echo $1 | sed "s#RLE#LIBVISO#g"`

./read_RLE_results_and_generate_lines_kml.sh $1
./read_RTK_results_and_generate_lines_kml.sh $RTK_string
#./read_LIBVISO_results_and_generate_lines_kml.sh $LIBVISO_string
