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

-module(eSBGE).
-export([start/3]).

-include_lib("wx/include/wx.hrl").
-include_lib("wx/include/gl.hrl").
-include("eSBGE.hrl").


start({Width,Height},GLCanvas,WXEnv) ->
    spawn_link(
      fun() -> 
	      wx:set_env(WXEnv),
	      init({Width,Height},GLCanvas)
      end).

init({Width,Height},GLCanvas)->
    false = wx:is_null(wxGLCanvas:getContext(GLCanvas)),
    wxGLCanvas:setCurrent(GLCanvas),
    glInit(Width,Height),
    self() ! draw,%start the drawing loop
    loop(#eSBGELoopInput{glCanvas = GLCanvas}).

glInit(W,H) ->
    gl:viewport(0,0,W,H),
    gl:matrixMode(?GL_PROJECTION),
    gl:loadIdentity(),
    gl:matrixMode(?GL_MODELVIEW),
    gl:loadIdentity(),
    gl:enable(?GL_DEPTH_TEST),
    gl:depthFunc(?GL_LESS),
    gl:clearColor(0.0, 0.0, 0.0, 1.0), % clearing the screens clears it to black
    gl:clearDepth(1.0),% clearing the depth buffers clears it to depth 1.0
    gl:enable(?GL_LIGHTING),
    gl:enable(?GL_LIGHT0),
    gl:shadeModel(?GL_SMOOTH),
    gl:enable(?GL_COLOR_MATERIAL),
    gl:lightModeli(?GL_LIGHT_MODEL_TWO_SIDE, ?GL_TRUE).

loop(#eSBGELoopInput{masterModelTableID = uninitialized,cameraTable=uninitialized}=Input) ->
    MasterModelTableID = ets:new(
			   masterModelTable,
			   [
			    set,
			    private,
			    {keypos,1}
			   ]),
    CameraTable = ets:new(
		    cameraTable,
		    [
		     set,
		     private,
		     {keypos,1}
		    ]),
    loop(Input#eSBGELoopInput{masterModelTableID = MasterModelTableID,cameraTable = CameraTable});

loop(#eSBGELoopInput{masterModelTableID = MasterModelTable,nextMasterModelID = NextMasterModelID,cameraTable = CameraTable,glCanvas = GLCanvas,timeStamp = TimeStamp}=Input)->
    receive
	draw->
	    timer:send_after(20,self(),draw),
	    gl:clear(?GL_DEPTH_BUFFER_BIT bor ?GL_COLOR_BUFFER_BIT),
	    CameraList = ets:tab2list(CameraTable),
	    lists:foreach(
	      fun({CameraPID})->
		      drawFromCamera(CameraPID,MasterModelTable,TimeStamp)
	      end,
	      CameraList),
	    wxGLCanvas:swapBuffers(GLCanvas),
	    %Diff = timer:now_diff(erlang:now(),TimeStamp),
	    %io:format("~B~n",[Diff]),
	    loop(Input#eSBGELoopInput{timeStamp = erlang:now()})
    after 0->
	    receive
		{loadedIFSModel,ModelID,PosSize,Positions,Normals,NumIndices} ->
		    VertexBufferID = flushBinaryToOpenGL(Positions,PosSize),
		    NormalBufferID = flushBinaryToOpenGL(Normals,PosSize),
		    ets:insert(MasterModelTable,{ModelID,VertexBufferID,NormalBufferID,NumIndices}),
		    loop(Input);
		{addCamera,CameraPID}->
		    ets:insert(CameraTable,{CameraPID}),
		    loop(Input);
		stop ->
		    ok;
		{From, {addModelFromIFSFile, Filename}}=RecievedMessage ->
		    ets:insert(MasterModelTable,{NextMasterModelID,uninitialized}),
		    SelfPID = self(),
		    spawn_link(
		      fun() ->
			      loadIFSModel(Filename,NextMasterModelID,SelfPID)
		      end),
		    From ! {self(), RecievedMessage, {ok,NextMasterModelID}},
		    loop(Input#eSBGELoopInput{nextMasterModelID = NextMasterModelID + 1});
		draw->
		    self() ! draw,%let the outer draw reciever handle it
		    loop(Input);
		Other->
		    error_logger:error_msg(
		      "Error: Process ~w got unknown msg ~w~n.", 
		      [self(), Other]),
		    loop(Input)
	    end
    end.
    

drawFromCamera(Camera,MasterModelTable,TimeStamp)->
    ToSend = {self(),{getDrawInfo,TimeStamp}},
    Camera ! ToSend,
    receive
	{Camera,ToSend,{DrawInfoList,#cameraInfo{vfov = VFOV,aRatio = ARatio,near=Near,far=Far,pos = {EyeX,EyeY,EyeZ}=Pos,orientation= Quaternion} = _CamInfo}}->
	    gl:matrixMode(?GL_PROJECTION),
	    gl:pushMatrix(),
	    glu:perspective(VFOV,ARatio,Near,Far),
	    gl:matrixMode(?GL_MODELVIEW),
	    gl:pushMatrix(),
	    {CenterX,CenterY,CenterZ} = vec3:add(Pos,vec3:rotateByQuaternion(camera:forward(),Quaternion)),
	    {UpX,UpY,UpZ} = vec3:rotateByQuaternion(camera:up(),Quaternion),
	    glu:lookAt(EyeX,EyeY,EyeZ,CenterX,CenterY,CenterZ,UpX,UpY,UpZ),
	    gl:lightfv(?GL_LIGHT0,?GL_POSITION,{0.0,0.0,0.0,1.0}),
	    lists:foreach(
	      fun(#instancedModelInfo{}=DrawInfo)->
		      drawInstancedModel(MasterModelTable,DrawInfo)
	      end,
	      DrawInfoList),
	    gl:matrixMode(?GL_MODELVIEW),
	    gl:popMatrix(),
	    gl:matrixMode(?GL_PROJECTION),
	    gl:popMatrix()
    end.
	
    
drawInstancedModel(MasterModelTable,#instancedModelInfo{isArena=IsArena,masterModelID=MasterModelID,pos={PosX,PosY,PosZ},scale={ScaleX,ScaleY,ScaleZ}=_Scale,orientation = Quaternion,color = {R,G,B}})->
    case ets:lookup(MasterModelTable,MasterModelID) of
	[{MasterModelID,uninitialized}] ->
	    ok;
	[{MasterModelID,PosBufferID,NormalBufferID,NumIndices}] ->
	    gl:pushMatrix(),
	    gl:translated(PosX,PosY,PosZ),
	    {{RX,RY,RZ},Angle} = quaternion:toAxisAngleinDegrees(Quaternion),
	    gl:rotated(Angle,RX,RY,RZ),
	    gl:scaled(ScaleX,ScaleY,ScaleZ), %for some reason, this causes a crash
	    gl:bindBuffer(?GL_ARRAY_BUFFER,PosBufferID),
	    gl:enableClientState(?GL_VERTEX_ARRAY),
	    gl:enableClientState(?GL_NORMAL_ARRAY),
	    gl:vertexPointer(3,?GL_FLOAT,0,0),
	    gl:bindBuffer(?GL_ARRAY_BUFFER,NormalBufferID),
	    gl:normalPointer(?GL_FLOAT,0,0),
	    gl:bindBuffer(?GL_ARRAY_BUFFER,0),
	    gl:color3d(R,G,B),
	    case IsArena of
		true->
		    gl:polygonMode(?GL_FRONT, ?GL_LINE),
		    gl:polygonMode(?GL_BACK, ?GL_LINE);
		false->
		    ok
	    end,
	    gl:drawArrays(?GL_TRIANGLES,0,NumIndices),
	    case IsArena of
		true->
		    gl:polygonMode(?GL_FRONT, ?GL_FILL),
		    gl:polygonMode(?GL_BACK, ?GL_FILL);
		false->
		    ok
	    end,
	    gl:popMatrix()
    end.

flushBinaryToOpenGL(Binary,SizeInBytes)->
    [BufferID] = gl:genBuffers(1),
    1 = gl:isBuffer(BufferID),
    gl:bindBuffer(?GL_ARRAY_BUFFER,BufferID),
    gl:bufferData(?GL_ARRAY_BUFFER,SizeInBytes,Binary,?GL_STATIC_DRAW),
    %gl:bindBuffer(?GL_ARRAY_BUFFER,0),
    BufferID.

loadIFSModel(Filename,ModelID,SendBackPid)->
    {
     {uniquePositions,_SizeVerts,UniquePositions},
     {indices,SizeIndices,Indices}
    } = brownMesh:load(Filename),
    Positions = expandPositionsFromIndices(UniquePositions,Indices),
    Normals = generateNonUniqueNormals(Positions),
    SendBackPid ! {loadedIFSModel,ModelID,3*SizeIndices,Positions,Normals,SizeIndices div 4}.

generateNonUniqueNormals(Positions)->
    generateNonUniqueNormals(Positions,<<>>).
generateNonUniqueNormals(<<>>,Normals)->
    Normals;
generateNonUniqueNormals(<< V0x:32/little-float , V0y:32/little-float , V0z:32/little-float , V1x:32/little-float , V1y:32/little-float , V1z:32/little-float , V2x:32/little-float , V2y:32/little-float , V2z:32/little-float , Positions/binary >>,Normals) ->
    CV0 = {-1.0*V0x,-1.0*V0y,-1.0*V0z},
    V1 = {V1x,V1y,V1z},
    V2 = {V2x,V2y,V2z},
    {Nx,Ny,Nz} = vec3:normal(vec3:cross(
		   vec3:add(V1,CV0),
		   vec3:add(V2,CV0))),
    generateNonUniqueNormals(Positions,<<Normals/binary,Nx:32/little-float,Ny:32/little-float,Nz:32/little-float,Nx:32/little-float,Ny:32/little-float,Nz:32/little-float,Nx:32/little-float,Ny:32/little-float,Nz:32/little-float>>).

expandPositionsFromIndices(Vertices,Indices)->
    expandPositionsFromIndices(Vertices,Indices,<<>>).

expandPositionsFromIndices(_Vertices,<<>>,ExpandedVertices)->
    ExpandedVertices;
expandPositionsFromIndices(Vertices,<<Index:32/little,Indices/binary>>,ExpandedVertices) ->
    VertexSize = 4*3, %3 floats per vertex, each float is a 4 byte float
    Offset = VertexSize*Index,
    << _:Offset/binary , Vector:VertexSize/binary , _/binary >> = Vertices,
    expandPositionsFromIndices( Vertices , Indices , << ExpandedVertices/binary , Vector/binary >> ).
