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

-module(mat3).

-compile(inline).

-export([mul/2,transpose/1,index/3]).

mul({R1x,R1y,R1z,
     R2x,R2y,R2z,
     R3x,R3y,R3z},
    {_Vx,_Vy,_Vz}=V)->
    R1 = {R1x,R1y,R1z},
    R2 = {R2x,R2y,R2z},
    R3 = {R3x,R3y,R3z},
    {vec3:dot(V,R1),vec3:dot(V,R2),vec3:dot(V,R3)};
mul({R1x,R1y,R1z,
     R2x,R2y,R2z,
     R3x,R3y,R3z},
    {C1x,C2x,C3x,
     C1y,C2y,C3y,
     C1z,C2z,C3z})->
    R1 = {R1x,R1y,R1z},
    R2 = {R2x,R2y,R2z},
    R3 = {R3x,R3y,R3z},
    C1 = {C1x,C1y,C1z},
    C2 = {C2x,C2y,C2z},
    C3 = {C3x,C3y,C3z},
    {vec3:dot(C1,R1),vec3:dot(C2,R1),vec3:dot(C3,R1),
     vec3:dot(C1,R2),vec3:dot(C2,R2),vec3:dot(C3,R2),
     vec3:dot(C1,R3),vec3:dot(C2,R3),vec3:dot(C3,R3)}.

transpose({R1x,R1y,R1z,
	   R2x,R2y,R2z,
	   R3x,R3y,R3z})->
    {R1x,R2x,R3x,
     R1y,R2y,R3y,
     R1z,R2z,R3z}.

index({V0,V1,V2,
       V3,V4,V5,
       V6,V7,V8},
      X,Y)->
    case X of
	0 ->
	    case Y of
		0 ->
		    V0;
		1 ->
		    V3;
		2 ->
		    V6
	    end;
	1 ->
	    case Y of
		0 ->
		    V1;
		1 ->
		    V4;
		2 ->
		    V7
	    end;
	2 ->
	    case Y of
		0 ->
		    V2;
		1 ->
		    V5;
		2 ->
		    V8
	    end
    end.
    
    
