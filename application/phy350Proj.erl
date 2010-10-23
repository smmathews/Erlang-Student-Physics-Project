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

-module(phy350Proj).
-export([run/0,run/2]).
-include("../graphics/eSBGE.hrl").
-include("../physics/eBBPE.hrl").
-include_lib("wx/include/wx.hrl").

-record(loopInfo,
	{
	  camera = none,
	  width = 640,
	  height = 480,
	  curMousePos = none,
	  leftDown = false
	 }).
run()->
    run(20,boxes).
run(Num,Type) ->
    {_Frame,GLCanvas} = initWindowAndGL({640,480}),
    Graphics = eSBGE:start({640,480},GLCanvas,wx:get_env()),
    Physics = eBBPE:start(),
    CreateMasterSphereMessage = {self(),{addModelFromIFSFile,"sphere.ifs"}},
    CreateMasterCubeMessage = {self(),{addModelFromIFSFile,"cube.ifs"}},
    Graphics ! CreateMasterSphereMessage,
    Graphics ! CreateMasterCubeMessage,
    receive
	{Graphics,CreateMasterSphereMessage,{ok,SphereMasterModelID}}->
	    ok
    end,
    receive
	{Graphics,CreateMasterCubeMessage,{ok,CubeMasterModelID}}->
	    ok
    end,
    OurGraphicsScene = graphicsScene:start(),
    OurCamera = camera:start(#cameraInfo{scene = OurGraphicsScene}),
    AddCameraMessage = {addCamera,OurCamera},
    Graphics ! AddCameraMessage,
    OurPhysicsScene = physicsScene:start(),
    Physics ! {addPhysicsScene,OurPhysicsScene},
    case Type of
	spheres->
	    makeArena(SphereMasterModelID,#sphereInfo{},OurGraphicsScene,OurPhysicsScene),
	    makeSpinningBodies(Num,SphereMasterModelID,#sphereInfo{},OurGraphicsScene,OurPhysicsScene);
	boxes->
	    makeArena(CubeMasterModelID,#boxInfo{},OurGraphicsScene,OurPhysicsScene),
	    makeSpinningBodies(Num,CubeMasterModelID,#boxInfo{},OurGraphicsScene,OurPhysicsScene)
    end,
    loop(#loopInfo{camera=OurCamera}),
    Graphics ! stop,
    ok.



loop(#loopInfo{camera=OurCamera,leftDown = LeftDown,curMousePos = CurMousePos,width=Width,height=Height}=Input) ->
    receive
	#wx{event=#wxMouse{type=motion,x=NewX,y=NewY}}->
	    if 
		LeftDown->%leftDown is true
		    case CurMousePos of
			none->
			    ok;
			{OldX,OldY}->
			    OurCamera ! {rotateWithMouse,{NewX - OldX,NewY - OldY},{Width,Height},1.0}%SENSITIVITY IS HACK!
		    end,
		    loop(Input#loopInfo{curMousePos = {NewX,NewY}});
		true->%leftDown is false
		    loop(Input)
	    end;
	#wx{event=#wxKey{type=key_down,keyCode=65}}->%a button is down
	    OurCamera ! {moveByCameraCoordinates,{-1.0,0.0,0.0}},
	    loop(Input);
	#wx{event=#wxKey{type=key_down,keyCode=68}}->%d button is down
	    OurCamera ! {moveByCameraCoordinates,{1.0,0.0,0.0}},
	    loop(Input);
	#wx{event=#wxKey{type=key_down,keyCode=83}}->%s button is down
	    OurCamera ! {moveByCameraCoordinates,{0.0,0.0,-5.0}},
	    loop(Input);
	#wx{event=#wxKey{type=key_down,keyCode=87}}->%w button is down
	    OurCamera ! {moveByCameraCoordinates,{0.0,0.0,5.0}},
	    loop(Input);
	#wx{event=#wxKey{type=key_down,keyCode=27}}->%esc buttin is down
	    ok;
	#wx{event=#wxClose{}}->
	    ok;
	#wx{event=#wxMouse{type=left_down}}->
	    loop(Input#loopInfo{leftDown = true});
	#wx{event=#wxMouse{type=left_up}}->
	    loop(Input#loopInfo{leftDown = false,curMousePos = none});
	#wx{event=#wxMouse{type=leave_window}}->
	     loop(Input#loopInfo{leftDown = false,curMousePos = none});
	Other->
	    error_logger:error_msg(
	      "Error: Phy350Proj loop Process ~w got unkown msg ~w~n",
	      [self(),Other])%,
	    %loop(Input)
    end.

makeSpinningBodies(0,_,_,_,_)->
    ok;
makeSpinningBodies(NumBodies,MasterModelID,RigidBodyGeometry,GraphicsScene,PhysicsScene) ->
    Orient = quaternion:normal(quaternion:fromAxisAngle({{random:uniform(),random:uniform(),random:uniform()},random:uniform()})),
    RigidBodyPID = rigidBody:start(
		     #rigidBodyInfo
		     {geometry=RigidBodyGeometry,
		      scene = PhysicsScene,
		      center = {(random:uniform() - 0.5)*20.0,(random:uniform() - 0.5)*20.0,(random:uniform() - 0.5)*20.0}%,
						%orientation = Orient
		     }),
    InstancedModelPID = instancedModel:start(
			  #instancedModelInfo
			  {masterModelID=MasterModelID,
			   physicsNodePID = RigidBodyPID,
			   color = vec3:normal({random:uniform(),random:uniform(),random:uniform()}),
						%to speed up, perhaps we should have the instancedModel keep AxisAngles instead of quaternions, or at least have them send it to us as Axis Angles
			   orientation = Orient
			  }),
    CreateInstancedMessage = {addInstancedModel,InstancedModelPID},
    CreateRigidBodyMessage = {addRigidBody,RigidBodyPID},
    GraphicsScene ! CreateInstancedMessage,
    PhysicsScene ! CreateRigidBodyMessage,
    makeSpinningBodies(NumBodies - 1, MasterModelID,RigidBodyGeometry,GraphicsScene,PhysicsScene).

makeArena(MasterModelID,#sphereInfo{}=Geometry,GraphicsScene,PhysicsScene) ->
    RigidBodyPID = rigidBody:start(
		     #rigidBodyInfo
		     {geometry=Geometry#sphereInfo{radius = 20.0},
		      scene = PhysicsScene,
		      center = {0.0,0.0,0.0},
		      unMovable=true
		     }
		    ),
    InstancedModelPID = instancedModel:start(
			  #instancedModelInfo
			  {masterModelID=MasterModelID,
			   physicsNodePID = RigidBodyPID,
			   color = vec3:normal({random:uniform(),random:uniform(),random:uniform()}),
			   scale = {20.0,20.0,20.0},
			   isArena = true,%to speed up, perhaps we should have the instancedModel keep AxisAngles instead of quaternions, or at least have them send it to us as Axis Angles
			   orientation = quaternion:normal(quaternion:fromAxisAngle({{random:uniform(),random:uniform(),random:uniform()},random:uniform()}))
			  }),
    CreateInstancedMessage = {addInstancedModel,InstancedModelPID},
    CreateRigidBodyMessage = {addRigidBody,RigidBodyPID},
    GraphicsScene ! CreateInstancedMessage,
    PhysicsScene ! CreateRigidBodyMessage;
makeArena(MasterModelID,#boxInfo{}=Geometry,GraphicsScene,PhysicsScene) ->
    RigidBodyPID = rigidBody:start(
		     #rigidBodyInfo
		     {geometry=Geometry#boxInfo{widthHalf = 10.0, heightHalf = 0.5, depthHalf=10.0},
		      scene = PhysicsScene,
		      center = {0.0,-20.0,0.0},
		      unMovable=true
		     }
		    ),
    InstancedModelPID = instancedModel:start(
			  #instancedModelInfo
			  {masterModelID=MasterModelID,
			   physicsNodePID = RigidBodyPID,
			   color = {random:uniform(),random:uniform(),random:uniform()},
			   scale = {20.0,1.0,20.0}
			  }),
    CreateInstancedMessage = {addInstancedModel,InstancedModelPID},
    CreateRigidBodyMessage = {addRigidBody,RigidBodyPID},
    GraphicsScene ! CreateInstancedMessage,
    PhysicsScene ! CreateRigidBodyMessage.

initWindowAndGL({Width,Height})->
    WX = wx:new(),
    Frame = wxFrame:new(WX,1,"phy350Proj",[{size,{Width,Height}}]),
    GLCanvas = wxGLCanvas:new(Frame,[{attribList,[1,4]},{size,{Width,Height}}]),
    ok = wxFrame:connect(Frame, close_window),
    ok = wxFrame:connect(GLCanvas, motion),
    ok = wxFrame:connect(GLCanvas, key_down),
    ok = wxFrame:connect(GLCanvas, left_down),
    ok = wxFrame:connect(GLCanvas, left_up),
    ok = wxFrame:connect(GLCanvas, leave_window),
    wxFrame:show(Frame), %% showing has to be done to initialize context
    false = wx:is_null(wxGLCanvas:getContext(GLCanvas)),
    wxGLCanvas:setCurrent(GLCanvas),
    {Frame,GLCanvas}.
