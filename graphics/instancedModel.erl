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

-module(instancedModel).
-export([start/1]).

-include("eSBGE.hrl").
-include("../physics/eBBPE.hrl").

start(#instancedModelInfo{}=Input)->
    spawn_link(fun()->
		       loop(Input)
	       end).

loop(#instancedModelInfo{}=Input)->
    receive
	{setPhysicsNode,PhysicsNodePID}->
	    loop(Input#instancedModelInfo{physicsNodePID=PhysicsNodePID});
	{From,{getDrawInfo,TimeStamp}}=ReceivedMessage->
	    #instancedModelInfo{physicsNodePID=PhysicsNodePID} = Input,
	    SentMessage = {self(),{getDrawInfo,TimeStamp}},
	    PhysicsNodePID ! SentMessage,
	    receive
		{PhysicsNodePID,SentMessage,#rigidBodyDrawInfo{position=Pos,orientation=Orient}}->
		    From ! {self(),ReceivedMessage,Input#instancedModelInfo{pos=Pos,orientation=Orient}}
	    end,
	    loop(Input);
	Other->
	    error_logger:error_msg(
	      "Error: Process ~w got unknown msg ~w~n.", 
	      [self(), Other]),
	    loop(Input)
    end.
    
