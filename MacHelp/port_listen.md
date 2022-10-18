#Mac helper
1) ls /dev/*
2) sudo minicom
sudo minicom -s
3) A - serial port mapping - change correct USB device and set as default
4) Open port



Roboti arhitektuuri ülevaade:

OLEKUD:
	Drive Forward 	- sõidab otse edasi
	Reverse 		- tagurdab veidi, siis naaseb Drive Forward olekusse

Drive Forward olek kehtib seni, kui keskmine sensor ei näita nt alla viie sentimeetri.
Kui keskmine sensor näitab väga lähedal takistust, siis liigib robot Reverse olekusse.

Reverse olekus robot tagurdab. Olek kehtib ettemääratud aja või keskmine sensor näitab piisavalt suurt kaugust.
Oleku lõppedes naaseb robot Drive Forward olekusse. 


Pööramine:
Vaatame vasakut ja paremat andurit. Näiteks vasak sensor näitab 10cm ja parem 20cm.
Sellisel juhul peaksime pöörama paremale. Pööramise tugevuse saab näiteks sensorite näitude vahest.
Pööramise tugevust saab kindlasti kuidagi ka normaliseerida.

