echo "<?xml version=\"1.0\" encoding=\"utf-8\"?>
<kml xmlns=\"http://www.opengis.net/kml/2.2\">
 <Document>
  <name>RTK GPS Points</name>" > $2


for file in $1/*
do tmp_line=`cat $file`
 read -a arr <<<$tmp_line
 echo "<Placemark><Point><coordinates>${arr[1]},${arr[0]},${arr[2]}</coordinates></Point></Placemark>"
done >> $2

echo  "
 </Document>
</kml>" >> $2
