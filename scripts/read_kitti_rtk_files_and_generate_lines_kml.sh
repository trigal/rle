echo "<?xml version=\"1.0\" encoding=\"utf-8\"?>
<kml xmlns=\"http://www.opengis.net/kml/2.2\">
 <Document>
  <name>RTK GPS lines</name>

	<Style id=\"sh_ylw-pushpin\">
		<IconStyle>
			<scale>1.2</scale>
		</IconStyle>
		<LineStyle>
			<color>ff00ff00</color>
			<width>4</width>
		</LineStyle>
	</Style>
	<Style id=\"sn_ylw-pushpin\">
		<LineStyle>
			<color>ff00ff00</color>
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
    <coordinates>" > $2

for file in $1/*
do tmp_line=`cat $file`
 read -a arr <<<$tmp_line
 echo -n "${arr[1]},${arr[0]},${arr[2]} "
done >> $2

echo "
    </coordinates>
   </LineString>
  </Placemark>
 </Document>
</kml>" >> $2
