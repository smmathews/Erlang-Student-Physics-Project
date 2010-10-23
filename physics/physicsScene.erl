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

-module(physicsScene).
-export([start/0]).
-include("eBBPE.hrl").

start()->
    spawn_link(fun()->
		       init()
	       end).

init()->
    PhysicsNodeTable = ets:new(
			 physicsNodeTable,
			 [
			  set,
			  public,
			  {keypos,1}
			 ]
			),
    loop(PhysicsNodeTable,erlang:now()).



loop(PhysicsNodeTable,TimeStamp1)->
    receive
	{addRigidBody,PhysicsNodePID}->
	    ets:insert(PhysicsNodeTable,{PhysicsNodePID}),
	    loop(PhysicsNodeTable,TimeStamp1);
	{From,{update,TimeStamp2}}=ReceivedMessage->
	    %spawn_link(
	     % fun()->
		      _RigidBodyList = update(PhysicsNodeTable,TimeStamp1,TimeStamp2),
	     % end),
	    %lists:foreach(
	    %  fun({PID,Info})->
		%      PID ! {self(),{update,Info}}
	      %end,
	      %RigidBodyList),
	    From ! {self(),ReceivedMessage,done},
	    loop(PhysicsNodeTable,TimeStamp2);
	Other->
	    error_logger:error_msg(
	      "Error: physicsScene:loop Process ~w got unknown msg ~w~n.",
	      [self(), Other]),
	    loop(PhysicsNodeTable,TimeStamp1)
    end.
    

updateAllRigidBodyInfo(RigidBodyList,TimeDiffSecs)->
    updateAllRigidBodyInfo(TimeDiffSecs,RigidBodyList,[]).

updateAllRigidBodyInfo(_TimeDiffSecs,[],NewList)->
    NewList;
updateAllRigidBodyInfo(TimeDiffSecs,[{PID,RigidBodyInfo} | RigidBodyList],NewList) ->
    updateAllRigidBodyInfo(TimeDiffSecs,RigidBodyList,[{PID,rigidBody:updateDiffTime(RigidBodyInfo,TimeDiffSecs)} | NewList]).


-define(FIXEDTIME,1.0/120.0).
-define(MINTIMESLICE,?FIXEDTIME/2.0).


updateFixedTimeSlice(RigidBodyList,FinalTime)
  when FinalTime > ?FIXEDTIME->
    NewRigidBodyList = updateRecurse(RigidBodyList, ?FIXEDTIME , ?FIXEDTIME , ?FIXEDTIME/2.0 ),
    lists:foreach(
      fun({PID,Info})->
	      PID ! {self(),{update,Info}}
      end,
      NewRigidBodyList),
    updateFixedTimeSlice(NewRigidBodyList,FinalTime - ?FIXEDTIME );
updateFixedTimeSlice(RigidBodyList,FinalTime) ->
    NewList = updateRecurse(RigidBodyList,FinalTime,FinalTime,FinalTime/2.0),
    lists:foreach(
      fun({PID,Info})->
	      PID ! {self(),{update,Info}}
      end,
      NewList),
    NewList.

updateRecurse(RigidBodyList,_TimeDiffSecs,FinalTimeDiff,_TimeToAddOrSubtract)
  when 0.0 >= FinalTimeDiff->
    case isInterpenetrating(RigidBodyList) of
	{true,InterpenetratingList,NonInterpenetratingList}->
	    lists:append(NonInterpenetratingList, InterpenetratingList);
	false->
	    RigidBodyList
    end;
updateRecurse(RigidBodyList,TimeDiffSecs,TimeDiffSecs,TimeToAddOrSubtract)
  when (TimeToAddOrSubtract < ?MINTIMESLICE)->%treat our ending time as the collision time, because we know there's an interpenetration
    FinalRigidList = updateAllRigidBodyInfo(RigidBodyList,TimeDiffSecs),    
    case isInterpenetrating(FinalRigidList) of
	{true,InterpenetratingList,NonInterpenetratingList}->
	    lists:append(NonInterpenetratingList, InterpenetratingList);
	false->
	    FinalRigidList
    end;    
updateRecurse(RigidBodyList,TimeDiffSecs,FinalTimeDiff,TimeToAddOrSubtract)
  when (TimeToAddOrSubtract < ?MINTIMESLICE)->%treat our ending time as the collision time, because we know there's an interpenetration
    case isInterpenetrating(updateAllRigidBodyInfo(RigidBodyList,TimeDiffSecs)) of
	{true,InterpenetratingList,NonInterpenetratingList}->
	    NewList = lists:append(NonInterpenetratingList, InterpenetratingList),
	    NewDiff = FinalTimeDiff - TimeDiffSecs;
	false->
	    {true,InterpenetratingList,NonInterpenetratingList} = isInterpenetrating(updateAllRigidBodyInfo(RigidBodyList,TimeDiffSecs + (2.0*TimeToAddOrSubtract))),
	    NewList = lists:append(NonInterpenetratingList, InterpenetratingList),
	    NewDiff = FinalTimeDiff - (TimeDiffSecs + (2.0*TimeToAddOrSubtract))
    end,
    updateRecurse(NewList,NewDiff,NewDiff,NewDiff/2.0);
updateRecurse(RigidBodyList,TimeDiffSecs,TimeDiffSecs,TimeToAddOrSubtract)->
    FinalRigidBodyList = updateAllRigidBodyInfo(RigidBodyList,TimeDiffSecs),
    case isInterpenetrating(FinalRigidBodyList) of
	{true,_FinalInterpenetratingList,_FinalNonInterpenetratingList}->
	    updateRecurse(RigidBodyList,TimeDiffSecs - TimeToAddOrSubtract,TimeDiffSecs,TimeToAddOrSubtract/2.0);
	false->
	    FinalRigidBodyList
    end;
%HACK NUMBER OF MAX RECURSES!
updateRecurse(RigidBodyList,TimeDiffSecs,FinalTimeDiff,TimeToAddOrSubtract)->
    FinalRigidBodyList = updateAllRigidBodyInfo(RigidBodyList,TimeDiffSecs),
    case isInterpenetrating(FinalRigidBodyList) of
	{true,_FinalInterpenetratingList,_FinalNonInterpenetratingList}->
	    updateRecurse(RigidBodyList,TimeDiffSecs - TimeToAddOrSubtract,FinalTimeDiff,TimeToAddOrSubtract/2.0);
	false->
	    updateRecurse(RigidBodyList,TimeDiffSecs + TimeToAddOrSubtract,FinalTimeDiff,TimeToAddOrSubtract/2.0)
    end.

update(PhysicsNodeTable,TimeStamp1,TimeStamp2)->
    TimeDiffSecs = timer:now_diff(TimeStamp2,TimeStamp1)/1000000.0,
    updateFixedTimeSlice(getAllRigidBodyInfo(ets:tab2list(PhysicsNodeTable)),TimeDiffSecs).
			  

%updateTimeStamp(_TimeStamp,[],NewList)->
%    NewList;
%updateTimeStamp(TimeStamp,[{PID,RigidBody} | RigidBodyList],NewList)->
%    updateTimeStamp(TimeStamp,RigidBodyList,[{PID,RigidBody#rigidBodyInfo{timeStamp=TimeStamp}} | NewList]).

%addDiffToNow(Seconds, {Mega,Sec,Micro})->
    

getAllRigidBodyInfo(PhysicsPIDList)->
    NumSent = sendAllInfoRequest(PhysicsPIDList,0),
    receiveAllInfo(NumSent).

sendAllInfoRequest([],NumSent)->
    NumSent;
sendAllInfoRequest([{PID} | PhysicsPIDList],NumSent) ->
    PID ! {self(),getInfo},
    sendAllInfoRequest(PhysicsPIDList,NumSent + 1).


receiveAllInfo(NumMessages)->
    receiveAllInfo({self(),getInfo},NumMessages,[]).

receiveAllInfo(_SentMessage,0,ListOfMessagesReceived)->
    ListOfMessagesReceived;
receiveAllInfo(SentMessage,NumMessages,ListOfMessagesReceived)->
    receive
	{From,SentMessage,Info}->
	    receiveAllInfo(SentMessage,NumMessages - 1,[{From,Info} | ListOfMessagesReceived])
    after 1000->
	    erlang:exit(
	      "Error: physicsScene Process ~w waited too long in receiveAllInfo~n.",
	      [self()])
    end.



receiveInterpenetrationAnswer(NumBodies)->
    receiveInterpenetrationAnswer(NumBodies,[],[]).

receiveInterpenetrationAnswer(0,[],_NonInterpenetrating)->
    false;
receiveInterpenetrationAnswer(0,Interpenetrating,NonInterpenetrating)->
    {true,Interpenetrating,NonInterpenetrating};
receiveInterpenetrationAnswer(NumBodies,Interpenetrating,NonInterpenetrating)->
    receive
	{false,RigidBody}->
	    receiveInterpenetrationAnswer(NumBodies - 1,Interpenetrating,[RigidBody | NonInterpenetrating]);
	{true,RigidBody}->
	    receiveInterpenetrationAnswer(NumBodies - 1,[RigidBody | Interpenetrating],NonInterpenetrating)
    after 1000->
	    erlang:exit(
	      "Error: physicsScene Process ~w waited too long in receiveInterpenetrationAnswer~n.",
	      [self()])
    end.

isInterpenetrating(RigidBodyInfoList)->
    isInterpenetrating(RigidBodyInfoList,RigidBodyInfoList,0).

isInterpenetrating(_OriginalList,[],NumBodies)->
    receiveInterpenetrationAnswer(NumBodies);
isInterpenetrating(OriginalList,[RigidBody | Tail ],NumBodies)->
    PID = self(),
    spawn_link(
      fun()->
	      isInterpenetrating(PID,RigidBody,OriginalList,[])
      end),
    isInterpenetrating(OriginalList,Tail,NumBodies + 1).

isInterpenetrating(PID,{_,_RigidBody1}=R1,[],[])->
    PID ! {false,R1};
isInterpenetrating(PID,{R1PID,RigidBody1}=_R1,[],CollisionPairs)->
    NewRigidBody1 = resolveAgainstBodies(RigidBody1,CollisionPairs),
    PID ! {true,{R1PID,NewRigidBody1}};%and send back updated R1, after resolving all collisions
isInterpenetrating(PID,{R1PID,RigidBody1}=R1,[{R1PID,RigidBody1}=R1 | RigidBodyList],CollisionPairs)->%checking interpenetration with itself
    isInterpenetrating(PID,R1,RigidBodyList,CollisionPairs);
isInterpenetrating(PID,{_R1PID,RigidBody1}=R1,[{_R2PID,RigidBody2}=R2 | RigidBodyList],CollisionPairs)->
    case rigidBody:isInterpenetrating(RigidBody1,RigidBody2) of
	false->
	    isInterpenetrating(PID,R1,RigidBodyList,CollisionPairs);
	true->
	    isInterpenetrating(PID,R1,RigidBodyList,[{R1,R2} | CollisionPairs])
    end.

resolveAgainstBodies(RigidBody1,[])->
    RigidBody1;
resolveAgainstBodies(RigidBody1,[{_,{_R2PID,RigidBody2}} | CollisionPairs])->
    resolveAgainstBodies(rigidBody:resolveFirstAgainstSecond(RigidBody1,RigidBody2),CollisionPairs).
