echo "<?xml version=\"1.0\" encoding=\"utf-8\"?>
<kml xmlns=\"http://www.opengis.net/kml/2.2\">
 <Document>
  <name>RLE lines</name>

	<Style id=\"sh_ylw-pushpin\">
		<IconStyle>
			<scale>1.2</scale>
		</IconStyle>
		<LineStyle>
			<color>ffff0000</color>
			<width>4</width>
		</LineStyle>
	</Style>
	<Style id=\"sn_ylw-pushpin\">
		<LineStyle>
			<color>ffff0000</color>
			<width>4</width>
		</LineStyle>
	</Style>
	<StyleMap id=\"msn_ylw-pushpin\">
		<Pair>
			<key>normal</key>
			<styleUrl>#sn_ylw-pushpin</styleUrl>
		</Pair>
		<Pair>
			<key>highlight</key>
			<styleUrl>#sh_ylw-pushpin</styleUrl>
		</Pair>
	</StyleMap>

  <Placemark id=\"track1\">
   <name>RTK GPS Path</name>
   <styleUrl>#msn_ylw-pushpin</styleUrl>
   <LineString>
    <coordinates>" > $1.kml

filename="$1"
while read -r line
do
    name=$line
    read -a arr <<<$line
    echo  "${arr[13]},${arr[12]}  "
done < "$filename" >> $1.kml

echo "
    </coordinates>
   </LineString>
  </Placemark>
 </Document>
</kml>" >> $1.kml
