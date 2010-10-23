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

-module(rigidBody).
-export([start/0,start/1,isInterpenetrating/2,updateDiffTime/2,resolveFirstAgainstSecond/2]).

-include("eBBPE.hrl").

-define(MAXSPEED,20.0).

start()->
    start(#rigidBodyInfo{}).
start(Input)->
    spawn_link(
      fun()->
	      loop(addInertiaToInfo(Input))
      end).

loop(#rigidBodyInfo{timeStamp=OriginalTimeStamp}=Input)->
    receive
	{From,{getDrawInfo,TimeStamp}}=ReceivedMessage->
	    SecondsDiff = timer:now_diff(TimeStamp,OriginalTimeStamp)/1000000.0,
	    #rigidBodyInfo{center=Pos,orientation=Orientation} = 
		if
		    SecondsDiff > 1.0/60.0->
			updateTimeStamp(Input,addSecondsToTimeStamp(OriginalTimeStamp,1.0/60.0));
		    true->
			updateTimeStamp(Input,TimeStamp)
		end,
	    From ! {self(),ReceivedMessage,#rigidBodyDrawInfo{position=Pos,orientation=Orientation}},
	    loop(Input);
	{_From,{update,NewInput}}->
	    loop(NewInput);
	{From,getInfo}=ReceivedMessage->
	    From ! {self(),ReceivedMessage,Input},
	    loop(Input);
	stop->
	    ok;
	Other->
	    error_logger:error_msg(
	      "Error: rigidBody:loop Process ~w got unkown msg ~w~n.",
	      [self(),Other]),
	    loop(Input)
    end.


%DOESN'T WORK WHEN DIFFERENCE IS MORE THAN 250 HOURS
addSecondsToTimeStamp({MegaSec,Sec,MicroSec},SecsDiff)->
    DiffInMicro = erlang:trunc(SecsDiff*1000000.0),
    MicroSec2 = MicroSec + (DiffInMicro rem 1000000),
    OverFlowMicro =
	if
	    MicroSec2 >= 1000000->
		true;
	    true ->
		false
	end,
    MicroSec3 = 
	if
	    OverFlowMicro->
		MicroSec2 - 1000000;
	    true ->
		MicroSec2
	end,
    Sec2 = 
	if
	    OverFlowMicro->
		Sec + erlang:trunc(SecsDiff) + 1;
	    true ->
		Sec + erlang:trunc(SecsDiff)
	end,
    OverFlowSec = 
	if
	    Sec2 >= 1000000->
		true;
	    true ->
		false
	end,
    Sec3 = 
	if
	    OverFlowSec->
		Sec2 - 1000000;
	    true ->
		Sec2
	end,
    MegaSec2 =
	if
	    OverFlowSec->
		MegaSec + 1;
	    true ->
		MegaSec
	end,
    {MegaSec2,Sec3,MicroSec3}.

updateTimeStamp(#rigidBodyInfo{timeStamp = TimeStamp1} = Input,TimeStamp2)->
    Secs = timer:now_diff(TimeStamp2,TimeStamp1)/1000000.0,
    NewInput = updateDiffTime(Input,Secs),
    NewInput#rigidBodyInfo{timeStamp=TimeStamp2}.


updateDiffTime(#rigidBodyInfo{unMovable=true,timeStamp=TimeStamp} = Input,Secs)->
    Input#rigidBodyInfo{timeStamp = addSecondsToTimeStamp(TimeStamp,Secs)};
updateDiffTime(#rigidBodyInfo{timeStamp=TimeStamp,center = Center1,mass = Mass,orientation = Orientation1,linearMomentum = LinearMomentum1,angularMomentum = AngularMomentum,inverseInertia = IBodyInv,inertia=IBody} = Input,Secs)->
    LinearMomentum2 = vec3:add(
			LinearMomentum1,
			vec3:mul(
			  {0.0,-9.81,0.0},%HACK GRAVITY
			  Secs)),
   MassInverse = 1.0/Mass,
    Pos = vec3:add(
	    Center1,
	    vec3:mul(
	      vec3:mul(
		LinearMomentum2,
		MassInverse),
	      Secs)),
    R = quaternion:toMatrix3(quaternion:normal(Orientation1)),
    Iinv = mat3:mul(R,mat3:mul(IBodyInv,mat3:transpose(R))),
    AngularVelocity = mat3:mul(Iinv,AngularMomentum),
    AngVelMag = vec3:magnitude(AngularVelocity),
    {ClampedAngularVelocity,ClampedAngularMomentum} = 
	if
	    AngVelMag > 4.0->
		A = vec3:mul(4.0,vec3:normal(AngularVelocity)),
		I = mat3:mul(R,mat3:mul(IBody,mat3:transpose(R))),
		{A,mat3:mul(I,A)};
	    true->
		{AngularVelocity,AngularMomentum}
	end,
    Spin = quaternion:mul(
	     0.5,
	     quaternion:mul(
	       vec3:toQuaternion(ClampedAngularVelocity),
	       Orientation1)),
    Orientation2 = quaternion:normal(
		     quaternion:add(
		       Orientation1,
		       quaternion:mul(
			 Spin,
			 Secs))),
    V1L = vec3:magnitude(vec3:mul(LinearMomentum2,1.0/Mass)),
    FinalMV = 
	if
	    V1L > ?MAXSPEED->
		vec3:mul(?MAXSPEED,vec3:normal(LinearMomentum2));
	    true->
		LinearMomentum2
	end,
    Input#rigidBodyInfo{timeStamp=addSecondsToTimeStamp(TimeStamp,Secs),center = Pos,orientation = Orientation2,linearMomentum = FinalMV,angularMomentum=ClampedAngularMomentum}.

closestPointinOBBtoPoint(#rigidBodyInfo{center=Center}=R1,Point)->
    D = vec3:add(Point , vec3:conjugate(Center)),
    Ret = Center,
    closestPointinOBBtoPoint(R1,D,Ret,0).
closestPointinOBBtoPoint(#rigidBodyInfo{},_D,Ret,3)->
    Ret;
closestPointinOBBtoPoint(#rigidBodyInfo{orientation=Orientation,geometry=#boxInfo{widthHalf=WH,heightHalf=HH,depthHalf=DH}}=R1,D,Ret,TestNum) ->
    {Axis,Extent} = 
	case TestNum of
	    0->
		{vec3:rotateByQuaternion({1.0,0.0,0.0},Orientation),WH};
	    1->
		{vec3:rotateByQuaternion({0.0,1.0,0.0},Orientation),HH};
	    2->
		{vec3:rotateByQuaternion({0.0,0.0,1.0},Orientation),DH}
	end,
    Dist = vec3:dot(D,Axis),
    NewDist = 
    if
	Dist < -1.0*Extent->
	    -1.0*Extent;
	Dist > Extent ->
	    Extent;
	true ->
	    Dist
    end,
    NewRet = vec3:add(Ret ,vec3:mul(NewDist,Axis )),
    closestPointinOBBtoPoint(R1,D,NewRet,TestNum+1).

getOBBRadiusVectorAgainstArena(#rigidBodyInfo{center=C1}=_R1,#rigidBodyInfo{center=_C2}=R2)->
    vec3:add(
      closestPointinOBBtoPoint(R2,C1),
      vec3:conjugate(C1)).
getOBBRadiusVectorAgainstInstance(#rigidBodyInfo{center=C1}=R1,#rigidBodyInfo{center=C2}=R2)->
    vec3:add(
      vec3:mul(
	0.5,
	vec3:add(
	  closestPointinOBBtoPoint(R1,C2),
	  closestPointinOBBtoPoint(R2,C1))),
      vec3:conjugate(C1)).

isInterpenetrating(#rigidBodyInfo{geometry=#sphereInfo{radius = Radius1},center = Center1},#rigidBodyInfo{geometry=#sphereInfo{radius = Radius2},center = Center2})->
    Diff    = vec3:add(Center2 , vec3:conjugate(Center1)),
    D2 = vec3:dot(Diff,Diff),
    R = Radius1 + Radius2,
    R2 = R*R,
    if
	D2 > R2->
	    false;%the sphere are completely seperate
	Radius1 < Radius2 ->
	    if
		D2 < (Radius2 - Radius1)*(Radius2 - Radius1)->
		    false;%sphere1 is inside sphere2, but not touching shells
		true->
		    true%they are Interpenetrating
	    end;
	true->
	    if
		D2 < (Radius1 - Radius2)*(Radius1 - Radius2)->
		    false;%sphere2 is inside sphere1, but not touching shells
		true->
		    true%they are Interpenetrating
	    end
    end;
isInterpenetrating(#rigidBodyInfo{geometry=#sphereInfo{}}=R1,#rigidBodyInfo{geometry=#boxInfo{}}=R2)->
    isInterpenetrating(R2,R1);
isInterpenetrating(#rigidBodyInfo{geometry=#boxInfo{}}=_R1,#rigidBodyInfo{geometry=#sphereInfo{}}=_R2)->
    false;
isInterpenetrating(#rigidBodyInfo{geometry=#boxInfo{}}=R1,#rigidBodyInfo{geometry=#boxInfo{}}=R2) ->
    case oBBOverlap(R1,R2) of
	false->
	    false;
	{true,_Axis}->
	    true
    end.

oBBOverlap(#rigidBodyInfo{orientation=O1,center=C1,geometry=#boxInfo{}=B1},#rigidBodyInfo{orientation=O2,center=C2,geometry=#boxInfo{}=B2})->
    %make rotation matrix to express R2 in R1's coordinate frame
    WV1 = vec3:rotateByQuaternion({1.0,0.0,0.0},O1),
    HV1 = vec3:rotateByQuaternion({0.0,1.0,0.0},O1),
    DV1 = vec3:rotateByQuaternion({0.0,0.0,1.0},O1),
    WV2 = vec3:rotateByQuaternion({1.0,0.0,0.0},O2),
    HV2 = vec3:rotateByQuaternion({0.0,1.0,0.0},O2),
    DV2 = vec3:rotateByQuaternion({0.0,0.0,1.0},O2),
    T = vec3:add(C2,vec3:conjugate(C1)),
    oBBOverlap({B1,B2,WV1,HV1,DV1,WV2,HV2,DV2,T},1000000.0,{0.0,0.0,0.0},0).
oBBOverlap(_,_,Axis,15)->%15 tests have been checked, and all are overlapping. we have a collision.
    {true,Axis};
oBBOverlap({#boxInfo{widthHalf=WH1,heightHalf=HH1,depthHalf=DH1},#boxInfo{widthHalf=WH2,heightHalf=HH2,depthHalf=DH2},WV1,HV1,DV1,WV2,HV2,DV2,T}=Input,MinOverlap,MinAxis,TestNum)->
    Axis = 
	case TestNum of
	    0 ->%axis WV1
		WV1;
	    1 ->%axis HV1
		HV1;
	    2 ->%axis DV1
		DV1;
	    3 ->%axis WV2
		WV2;
	    4 ->%axis HV2
		HV2;
	    5 ->%axis DV2
		DV2;
	    6 ->%axis WV1 x WV2
		vec3:normal(vec3:cross(WV1,WV2));
	    7 ->%axis WV1 x HV2
		vec3:normal(vec3:cross(WV1,HV2));
	    8 ->%axis WV1 x DV2
		vec3:normal(vec3:cross(WV1,DV2));
	    9 ->%axis HV1 x WV2
		vec3:normal(vec3:cross(HV1,WV2));
	    10 ->%axis HV1 x HV2
		vec3:normal(vec3:cross(HV1,HV2));
	    11 ->%axis HV1 x DV2
		vec3:normal(vec3:cross(HV1,DV2));
	    12 ->%axis DV1 x WV2
		vec3:normal(vec3:cross(DV1,WV2));
	    13 ->%axis DV1 x HV2
		vec3:normal(vec3:cross(DV1,HV2));
	    14 ->%axis DV1 x DV2
		vec3:normal(vec3:cross(DV1,DV2))
	end,
    R1 = WH1*(abs(vec3:dot(WV1,Axis))) + HH1*(abs(vec3:dot(HV1,Axis))) + DH1*(abs(vec3:dot(DV1,Axis))),
    R2 = WH2*(abs(vec3:dot(WV2,Axis))) + HH2*(abs(vec3:dot(HV2,Axis))) + DH2*(abs(vec3:dot(DV2,Axis))),
    Overlap = abs(vec3:dot(T,Axis)) - (R1 + R2),
    case (Overlap > 0.0) of
	true ->%seperating axis found. no overlap.
	    false;
	false ->%no seperating axis found this time. check next one.
	    case ((abs(MinOverlap) > abs(Overlap)) and not (Axis == {0.0,0.0,0.0}))of
		true->
		    oBBOverlap(Input,Overlap,Axis,TestNum+1);
		false->
		    oBBOverlap(Input,MinOverlap,MinAxis,TestNum+1)
	    end
    end.


resolveFirstAgainstSecond(#rigidBodyInfo{unMovable = true}=R1,_R2)->%unomvable object, don't update
    R1;
resolveFirstAgainstSecond(#rigidBodyInfo{linearMomentum = MV1,geometry=#sphereInfo{},center = Center1}=R1,#rigidBodyInfo{geometry=#sphereInfo{},center = Center2,unMovable=true,linearMomentum=MV2}=R2)->%when hitting against arena, negate momentum in intersection direction
    N1 = vec3:normal(vec3:add(Center1, vec3:conjugate(Center2))),
    VDot = vec3:dot(N1,vec3:add(MV1,vec3:conjugate(MV2))),
    NormalOfIntersection = 
	if
	    0.0 >= VDot->
		N1;
	    true->
		vec3:conjugate(N1)
	end,
    NewMV1 = vec3:add(
	       MV1,
	       vec3:mul(
		 NormalOfIntersection,
		 -2.0*vec3:dot(MV1,NormalOfIntersection))),
    #rigidBodyInfo{center=NewPos} = resolveFirstAgainstSecond(R1,R2#rigidBodyInfo{unMovable=false}),
    R1#rigidBodyInfo{linearMomentum=NewMV1,center=NewPos};
resolveFirstAgainstSecond(#rigidBodyInfo{orientation=_O1,angularMomentum=AngularMomentum1,linearMomentum=MV1,geometry=#boxInfo{}}=R1,#rigidBodyInfo{geometry=#boxInfo{},linearMomentum=MV2,unMovable=true}=R2)->
    {true,N1} = oBBOverlap(R1,R2),
    VDot = vec3:dot(N1,vec3:add(MV1,vec3:conjugate(MV2))),
    NormalOfIntersection = 
	if
	    0.0 >= VDot->
		N1;
	    true->
		vec3:conjugate(N1)
	end,
    NewMV1 = vec3:add(
	       MV1,
	       vec3:mul(
		 NormalOfIntersection,
		 -2.0*vec3:dot(MV1,NormalOfIntersection))),
    LinearImpulse = vec3:add(NewMV1,vec3:conjugate(MV1)),
    RadiusVector = getOBBRadiusVectorAgainstArena(R1,R2),
    AngularImpulse = vec3:mul(1.000,vec3:cross(RadiusVector,LinearImpulse)),
    NewAngularMomentum = vec3:add(AngularMomentum1,AngularImpulse),
    %io:format("~p~n",[vec3:add(POI,vec3:conjugate(C1))]),
    R1#rigidBodyInfo{linearMomentum=NewMV1,angularMomentum=NewAngularMomentum};
resolveFirstAgainstSecond(#rigidBodyInfo{mass=M1,linearMomentum = MV1,geometry=#sphereInfo{radius=Radius1},center = Center1}=R1,#rigidBodyInfo{geometry=#sphereInfo{radius=Radius2},center = Center2,linearMomentum=MV2})->
    Diff    = vec3:add(Center2 , vec3:conjugate(Center1)),
    N1 = vec3:normal(vec3:add(Center1, vec3:conjugate(Center2))),
    VDot = vec3:dot(N1,vec3:add(MV1,vec3:conjugate(MV2))),
    NormalOfIntersection = 
	if
	    0.0 >= VDot->
		N1;
	    true->
		vec3:conjugate(N1)
	end,
    NewMV1 = vec3:add(
	       MV1,
	       vec3:add(
		   vec3:mul(
		     NormalOfIntersection,
		     vec3:dot(MV2,NormalOfIntersection)),
		vec3:mul(
		   NormalOfIntersection,
		   -1.0*vec3:dot(MV1,NormalOfIntersection)))),
    V1 = vec3:magnitude(vec3:mul(NewMV1,1.0/M1)),
    FinalMV1 = 
	if
	    V1 > ?MAXSPEED->
		vec3:mul(?MAXSPEED,vec3:normal(NewMV1));
	    true->
		NewMV1
	end,
    %R = Radius1 + Radius2,
    %R2 = R*R,
    D2 = vec3:dot(Diff,Diff),
    NewPos = 
	if
	    D2 < (Radius2 - Radius1 + 0.1)*(Radius2 - Radius1 + 0.1), Radius1 < Radius2->% sphere1 inside sphere2
		vec3:add(
		  Center2 ,
		  vec3:conjugate(
		    vec3:mul(Radius2 - Radius1 - 0.1,NormalOfIntersection)));
	    D2 < (Radius1 - Radius2)*(Radius1 - Radius2), Radius2 < Radius1->%sphere2 inside sphere1
		vec3:add(
		  Center2 ,
		  vec3:conjugate(
		    vec3:mul(Radius2 - Radius1 + 0.01,NormalOfIntersection)));
	    true->%sphere1 and sphere2 are disjoint is definetly outside of sphere
		vec3:add(
		  Center2,
		  vec3:mul((Radius1+Radius2),NormalOfIntersection))
	end,
    R1#rigidBodyInfo{linearMomentum=FinalMV1,center=NewPos};%center=NewPos}.
resolveFirstAgainstSecond(#rigidBodyInfo{orientation=_Orientation1,angularMomentum=AngularMomentum1,mass=M1,linearMomentum=MV1,geometry=#boxInfo{}=_Box1}=R1,#rigidBodyInfo{geometry=#boxInfo{},linearMomentum=MV2}=R2)->
    {true,N1} = oBBOverlap(R1,R2),
    VDot = vec3:dot(N1,vec3:add(MV1,vec3:conjugate(MV2))),
    NormalOfIntersection = 
	if
	    0.0 >= VDot->
		N1;
	    true->
		vec3:conjugate(N1)
	end,
    NewMV1 = vec3:add(
	       MV1,
	       vec3:add(
		   vec3:mul(
		     NormalOfIntersection,
		     vec3:dot(MV2,NormalOfIntersection)),
		vec3:mul(
		   NormalOfIntersection,
		   -1.0*vec3:dot(MV1,NormalOfIntersection)))),
    V1 = vec3:magnitude(vec3:mul(NewMV1,1.0/M1)),
    FinalMV1 = 
	if
	    V1 > ?MAXSPEED->
		vec3:mul(?MAXSPEED,vec3:normal(NewMV1));
	    true->
		NewMV1
	end,
    LinearImpulse = vec3:add(FinalMV1,vec3:conjugate(MV1)),
    RadiusVector = getOBBRadiusVectorAgainstInstance(R1,R2),
    AngularImpulse = vec3:cross(RadiusVector,LinearImpulse),
    NewAngularMomentum = vec3:add(AngularMomentum1,AngularImpulse),
    R1#rigidBodyInfo{linearMomentum=FinalMV1,angularMomentum=NewAngularMomentum};
resolveFirstAgainstSecond(R1,_R2) ->
    R1#rigidBodyInfo{linearMomentum={0.0,0.0,0.0}}.


addInertiaToInfo(#rigidBodyInfo{geometry = #sphereInfo{radius=Radius},mass = Mass}=Input)->
    NonZero = 2.0*Mass*Radius*Radius/5.0,
    InvNonZero = 1.0/NonZero,
    Inertia = 
	{ NonZero, 0.0, 0.0,
	  0.0, NonZero, 0.0,
	  0.0, 0.0, NonZero
	 },
    InverseInertia = 
	{ InvNonZero, 0.0, 0.0,
	  0.0, InvNonZero, 0.0,
	  0.0, 0.0, InvNonZero
	 },
    Input#rigidBodyInfo{inertia = Inertia, inverseInertia = InverseInertia};
addInertiaToInfo(#rigidBodyInfo{geometry=#boxInfo{widthHalf=WH,heightHalf=HH,depthHalf=DH},mass=Mass}=Input) ->
    X = Mass*(HH*HH + DH*DH)/3.0,
    Y = Mass*(WH*WH + DH*DH)/3.0,
    Z = Mass*(WH*WH + HH*HH)/3.0,
    Inertia = 
	{ X, 0.0, 0.0,
	  0.0, Y, 0.0,
	  0.0, 0.0, Z
	 },
    InverseInertia = 
	{ 1.0/X, 0.0, 0.0,
	  0.0, 1.0/Y, 0.0,
	  0.0, 0.0, 1.0/Z
	 },
    Input#rigidBodyInfo{inertia = Inertia, inverseInertia = InverseInertia}.
