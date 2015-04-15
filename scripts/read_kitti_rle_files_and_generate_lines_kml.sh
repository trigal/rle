echo "<?xml version=\"1.0\" encoding=\"utf-8\"?>
<kml xmlns=\"http://www.opengis.net/kml/2.2\">
 <Document>
  <name>RLE lines</name>

	<Style id=\"sh_ylw-pushpin\">
		<IconStyle>
			<scale>1.2</scale>
		</IconStyle>
		<LineStyle>
			<color>ff0000ff</color>
			<width>4</width>
		</LineStyle>
	</Style>
	<Style id=\"sn_ylw-pushpin\">
		<LineStyle>
			<color>ff0000ff</color>
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
   <name>RLE Path</name>
   <styleUrl>#msn_ylw-pushpin</styleUrl>
   <LineString>
    <coordinates>" > $2

while read tmp_line
do
 read -a arr <<<$tmp_line
 echo  "${arr[13]},${arr[12]},0.0 "
done < $1 >> $2

echo "
    </coordinates>
   </LineString>
  </Placemark>
 </Document>
</kml>" >> $2
