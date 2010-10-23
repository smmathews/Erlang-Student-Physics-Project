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

-module(graphicsScene).
-export([start/0]).
-include("eSBGE.hrl").

start()->
    spawn_link(fun()->
		       init()
	       end).

init()->
    InstancedModelTable = ets:new(
			     instancedModelTable,
			     [
			      set,
			      private,
			      {keypos,1}
			     ]
			    ),
    LightsTable = ets:new(
		    lightsTable,
		    [
		     set,
		     private,
		     {keypos,1}
		    ]),
    loop(#graphicsSceneInput{instancedModelTable = InstancedModelTable , lightsTable = LightsTable}).

loop(#graphicsSceneInput{instancedModelTable = InstancedModelTable} = Input)->
    receive
	{addInstancedModel,InstanceModelPID}->
	    ets:insert(InstancedModelTable,{InstanceModelPID}),
	    loop(Input);
	{From,{getDrawInfo,TimeStamp}}=ReceivedMessage->
	    From ! {self(),ReceivedMessage,getDrawInfo(InstancedModelTable,TimeStamp)},
	    loop(Input);
	Other->
	    error_logger:error_msg(
	      "Error: Process ~w got unknown msg ~w~n.",
	      [self(), Other]),
	    loop(Input)
    end.

getDrawInfo(InstancedModelTable,TimeStamp)->
    NumMessages = sendAllDrawInfo(ets:tab2list(InstancedModelTable),TimeStamp),
    receiveAllDrawInfo(NumMessages,TimeStamp).

receiveAllDrawInfo(NumMessages,TimeStamp)->
    receiveAllDrawInfo({self(),{getDrawInfo,TimeStamp}},NumMessages,[]).

receiveAllDrawInfo(_SentMessage,0,ListOfMessagesReceived)->
    ListOfMessagesReceived;
receiveAllDrawInfo(SentMessage,NumMessages,ListOfMessagesReceived)->
    receive
	{_From,SentMessage,#instancedModelInfo{}=DrawInfo}->
	    receiveAllDrawInfo(SentMessage,NumMessages - 1,[DrawInfo | ListOfMessagesReceived])
    end.

sendAllDrawInfo(InstancedModelList,TimeStamp)->
    sendAllDrawInfo(TimeStamp,InstancedModelList,0).

sendAllDrawInfo(_TimeStamp,[],NumSent)->
    NumSent;
sendAllDrawInfo(TimeStamp,[{InstanceModelPID} | InstancedModelList],NumSent)->
    InstanceModelPID ! {self(),{getDrawInfo,TimeStamp}},
    sendAllDrawInfo(TimeStamp,InstancedModelList,NumSent + 1).
