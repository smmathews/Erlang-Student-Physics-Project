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

-module(camera).

-export([start/1,right/0,up/0,forward/0]).

-include("eSBGE.hrl").

start(#cameraInfo{}=Input)->
    spawn_link(fun()->
		       init(Input)
	       end).

init(#cameraInfo{}=Input)->
    loop(Input).

loop(#cameraInfo{scene = Scene}=Input)->
    receive
	{moveByCameraCoordinates,ToMove}->
	    loop(moveByCameraCoordinates(Input,ToMove));
	{rotateWithMouse,{_DX,_DY}=DMouse,{_Width,_Height}=ScreenDimens,Sensitivity}->
	    loop(rotateWithMouse(Input,DMouse,ScreenDimens,Sensitivity));
	{From,{getDrawInfo,TimeStamp}}=ReceivedMessage->
	    From ! {self(),ReceivedMessage,getDrawInfo(Scene,Input,TimeStamp)},
	    loop(Input);
	Other->
	    error_logger:error_msg(
	      "Error: Process ~w got unknown msg ~w~n.",
	      [self(), Other]),
	    loop(Input)
    end.

getDrawInfo(Scene,#cameraInfo{} = CamInfo,TimeStamp)->
    ToSend = {self(),{getDrawInfo,TimeStamp}},
    Scene ! ToSend,
    receive
	{Scene,ToSend,DrawInfoList}->
	    {DrawInfoList,CamInfo}
    end.

right()->
    {1.0,0.0,0.0}.
up()->
    {0.0,1.0,0.0}.
forward()->
    {0.0,0.0,-1.0}.

newRight(Orientation)->    
    vec3:rotateByQuaternion(right(),Orientation).

newUp(Orientation)->
    vec3:rotateByQuaternion(up(),Orientation).

newForward(Orientation)->
    vec3:rotateByQuaternion(forward(),Orientation).

moveByCameraCoordinates(#cameraInfo{pos = Pos,orientation=Orientation}=CamInfo,{DRight,DUp,DForward})->
    NewPos = vec3:add(
	       vec3:add(
		 vec3:add(
		   Pos , 
		   vec3:mul(newRight(Orientation),DRight)),
		 vec3:mul(newUp(Orientation),DUp)),
	       vec3:mul(newForward(Orientation),DForward)),
    CamInfo#cameraInfo{pos = NewPos}.

%Sensitivity from 0.0(No sensitivity) to 1.0(Super Sensitivity)
rotateWithMouse(#cameraInfo{orientation=Orientation}=CamInfo,{DX,DY},{Width,Height},Sensitivity)->
    RotateAboutUp = -1.0*(DX/Width)*Sensitivity,
    RotateAboutRight = -1.0*(DY/Height)*Sensitivity,
    NewOrientation = 
	quaternion:mul(
	  quaternion:mul(
	    quaternion:fromAxisAngle({newUp(Orientation),RotateAboutUp}),
	    quaternion:fromAxisAngle({newRight(Orientation),RotateAboutRight})),
	  Orientation),
    CamInfo#cameraInfo{orientation=NewOrientation}.
    
