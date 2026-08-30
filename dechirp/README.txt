https://share.google/aimode/6uXjbDO7qFYJuvZ02

rosep@MSI:/mnt/c/Users/rosep/Downloads/Chirp$


# generate chirp.txt  
python3 chirp_gen.py
 
# examine
python3 ../AFSK/spectrogram_analysis.py chirp.txt  44100 6000 8000
dechirp/Screenshot 2026-08-29 222747.png

# dechirp artificial chirp
python3 dechirp.py chirp.txt
dechirp/Screenshot 2026-08-29 184148.png


# dechirp real rx chirp
python3 dechirp.py ../AFSK/rx_chirp.txt
dechirp/Screenshot 2026-08-29 184315.png