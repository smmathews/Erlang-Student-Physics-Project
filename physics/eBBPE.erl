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

-module(eBBPE).
-export([start/0]).

start()->
    spawn_link(
      fun() ->
	      init()
      end).

init()->
    PhysicsSceneTable = ets:new(
			  physicsSceneTable,
			  [
			   set,
			   private,
			   {keypos,1}
			  ]),
    self() ! update,
    loop(PhysicsSceneTable).

loop(PhysicsSceneTable) ->
    receive
	update->
	    %self() ! update,
	    timer:send_after(20,self(),update),
	    update(PhysicsSceneTable,erlang:now()),
	    loop(PhysicsSceneTable)
    after 0->
	    receive
		{addPhysicsScene,ScenePID}->
		    ets:insert(PhysicsSceneTable,{ScenePID}),
		    loop(PhysicsSceneTable);
		update->
		    self() ! update,
		    loop(PhysicsSceneTable);
		Other->
		    error_logger:error_msg(
		      "Error: Process ~w got unkown msg ~w~n.",
		      [self(),Other]),
		    loop(PhysicsSceneTable)
	    end
    end.

update(PhysicsSceneTable,TimeStamp)->
    NumMessages = sendAllUpdateMessages(ets:tab2list(PhysicsSceneTable),TimeStamp),
    receiveAllUpdateConfirms({self(),{update,TimeStamp}},NumMessages).

receiveAllUpdateConfirms(_SentMessage,0)->
    ok;
receiveAllUpdateConfirms(SentMessage,NumMessages)->
    receive
	{_From,SentMessage,done}->
	    receiveAllUpdateConfirms(SentMessage,NumMessages - 1)
    end.

sendAllUpdateMessages(PhysicsSceneList,TimeStamp)->
    sendAllUpdateMessages(TimeStamp,PhysicsSceneList,0).

sendAllUpdateMessages(_TimeStamp,[],NumSent)->
    NumSent;
sendAllUpdateMessages(TimeStamp,[{PhysicsScenePID} | PhysicsSceneList],NumSent)->
    PhysicsScenePID ! {self(),{update,TimeStamp}},
    sendAllUpdateMessages(TimeStamp,PhysicsSceneList,NumSent + 1).
