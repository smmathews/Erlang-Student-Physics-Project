%%% Copyright (c) 2010 shane.m.mathews@gmail.com
%%% 
%%% Permission is hereby granted, free of charge, to any person obtaining a copy
%%% of this software and associated documentation files (the "Software"), to deal
%%% in the Software without restriction, including without limitation the rights
%%% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
%%% copies of the Software, and to permit persons to whom the Software is
%%% furnished to do so, subject to the following conditions:
%%% 
%%% The above copyright notice and this permission notice shall be included in
%%% all copies or substantial portions of the Software.
%%% 
%%% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
%%% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
%%% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
%%% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
%%% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
%%% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
%%% THE SOFTWARE.

-record(eSBGELoopInput,
	{
	  masterModelTableID=uninitialized,
	  nextMasterModelID=1,
	  cameraTable=uninitialized,
	  glCanvas,
	  timeStamp = erlang:now()
	 }
       ).

-record(instancedModelInfo,
	{
	  masterModelID = ugliestModelEver,
	  pos = { 0.0 , 0.0 , 0.0 },
	  scale = {1.0,1.0,1.0},
	  orientation = { 1.0 , {0.0 , 0.0 , 0.0 }},
	  physicsNodePID = none,
	  color = {1.0,1.0,1.0},
	  isArena = false
	 }
       ).

%assume without orientation that right is (1,0,0), up is (0,1,0), and forward is (0,0,-1) 
-record(cameraInfo,
	{
	  pos = {0.0,12.0,0.0},
	  orientation = quaternion:fromAxisAngle({{1.0,0.0,0.0},-3.14159/2.0}),
	  vfov = 45.0,
	  aRatio = 4.0/3.0,
	  near = 1.0,
	  far = 100.0,
	  scene = none
	  }
	).

-record(graphicsSceneInput,
	{
	  instancedModelTable = none,
	  lightsTable = none
	 }).


-record(lightInfo,
	{
	  ambient = {0.0,0.0,0.0,1.0},
	  diffuse = {0.0,0.0,0.0,1.0},
	  specular= {0.0,0.0,0.0,1.0},
	  pos = {0.0,0.0,0.0},
	  %overwrite the spot values to make a spotlight
	  spotExponent = 0.0,
	  spotCutOff = 180.0,
	  constAttenuation = 1.0,
	  linearAttenuation = 0.0,
	  quadAttenuation = 0.0
	 }).
