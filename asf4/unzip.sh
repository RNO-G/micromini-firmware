#! /bin/sh 

unzip "micromini.atzip"

#for some reason everything here has the wrong time
find . -type f -exec touch {} + 
