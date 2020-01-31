#!/bin/bash

cd ./SFND_2D_Feature_Tracking/build/

i=1

./2D_feature_tracking SHITOMASI BRISK $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking SHITOMASI BRIEF $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking SHITOMASI ORB $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking SHITOMASI FREAK $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking SHITOMASI SIFT $i >> ~/Desktop/new.txt 

let i++
./2D_feature_tracking FAST BRISK $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking FAST BRIEF $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking FAST ORB $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking FAST FREAK $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking FAST SIFT $i >> ~/Desktop/new.txt 

let i++
./2D_feature_tracking BRISK BRISK $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking BRISK BRIEF $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking BRISK ORB $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking BRISK FREAK $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking BRISK SIFT $i >> ~/Desktop/new.txt 

let i++
./2D_feature_tracking ORB BRISK $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking ORB BRIEF $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking ORB ORB $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking ORB FREAK $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking ORB SIFT $i >> ~/Desktop/new.txt 

let i++
./2D_feature_tracking AKAZE BRISK $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking AKAZE BRIEF $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking AKAZE ORB $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking AKAZE FREAK $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking AKAZE AKAZE $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking AKAZE SIFT $i >> ~/Desktop/new.txt 

let i++
./2D_feature_tracking SIFT BRISK $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking SIFT BRIEF $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking SIFT FREAK $i >> ~/Desktop/new.txt 
let i++
./2D_feature_tracking SIFT SIFT $i >> ~/Desktop/new.txt

cd ../..