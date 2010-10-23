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

-module(vec3).

-compile(inline).

-export([rotateByQuaternion/2,add/2,mul/2,dot/2,toQuaternion/1,conjugate/1,cross/2,normal/1,magnitude/1]).

toQuaternion({X,Y,Z})
   when is_float(X),is_float(Y),is_float(Z)->
    {0.0,{X,Y,Z}}.

rotateByQuaternion(V,Q)->
    {_,{X,Y,Z}} = quaternion:mul(quaternion:mul(Q,toQuaternion(V)),quaternion:conjugate(Q)),
    {X,Y,Z}.

add({X1,Y1,Z1},{X2,Y2,Z2})
  when is_float(X1),is_float(Y1),is_float(Z1),
       is_float(X2),is_float(Y2),is_float(Z2)->
    {X1+X2,Y1+Y2,Z1+Z2}.

mul({X,Y,Z},S)
  when is_float(X),is_float(Y),is_float(Z),is_float(S)->
    {X*S,Y*S,Z*S};
mul(S,{X,Y,Z})
  when is_float(X),is_float(Y),is_float(Z),is_float(S)->
    {X*S,Y*S,Z*S}.

dot({X1,Y1,Z1},{X2,Y2,Z2})
  when is_float(X1),is_float(Y1),is_float(Z1),
       is_float(X2),is_float(Y2),is_float(Z2)->
    X1*X2 + Y1*Y2 + Z1*Z2.

conjugate({X,Y,Z})
  when is_float(X),is_float(Y),is_float(Z)->
    {-1.0*X,-1.0*Y,-1.0*Z}.

cross({X1,Y1,Z1},{X2,Y2,Z2})
  when is_float(X1),is_float(Y1),is_float(Z1),
       is_float(X2),is_float(Y2),is_float(Z2)->
    {Y1*Z2 - Z1*Y2,Z1*X2 - X1*Z2,X1*Y2 - Y1*X2}.

normal({0.0,0.0,0.0}=Input)->
    Input;
normal(Input)->
    Den = 1.0/magnitude(Input),
    vec3:mul(Input,Den).

magnitude({X,Y,Z})
  when is_float(X),is_float(Y),is_float(Z)->
    math:sqrt(X*X + Y*Y + Z*Z).
