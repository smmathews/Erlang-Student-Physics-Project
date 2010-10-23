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

-record(sphereInfo,
	{
	  radius = 1.0
	 }
       ).

-record(boxInfo,
	{
	  widthHalf = 0.5,
	  heightHalf = 0.5,
	  depthHalf = 0.5
	 }
       ).

-define(WidthAxis,{1.0,0.0,0.0}).
-define(HeightAxis,{0.0,1.0,0.0}).
-define(DepthAxis,{0.0,0.0,1.0}).

-record(rigidBodyInfo,
	{
	  geometry=#sphereInfo{},
	  center = {0.0,0.0,0.0},
	  mass = 2.0,
	  orientation = {1.0,{0.0,0.0,0.0}},
	  linearMomentum = {0.0,0.0,0.0},
	  angularMomentum = {0.0,0.0,0.0},
	  timeStamp = erlang:now(),
	  inertia = none,
	  inverseInertia = none,
	  scene = none,
	  unMovable = false
	 }
       ).

-record(rigidBodyDrawInfo,
	{
	  position = {0.0,0.0,0.0},
	  orientation = {1.0,{0.0,0.0,0.0}}
	 }
       ).
