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

-module(quaternion).
-export([fromAxisAngle/1,toMatrix3/1,toAxisAngleinRadians/1,toAxisAngleinDegrees/1,normal/1,mul/2,conjugate/1,add/2]).
-compile(inline).

fromAxisAngle({{X,Y,Z},Theta})
   when is_float(X),is_float(Y),is_float(Z),is_float(Theta)->
    T2 = Theta/2.0,
    Sin2 = math:sin(T2),
    {math:cos(T2),{X*Sin2,Y*Sin2,Z*Sin2}}.

toAxisAngleinRadians({R,{I,J,K}}=Input)
  when is_float(I),is_float(J),is_float(K),is_float(R)->
    if
	R > 1.0 ->
	    {R2,{I2,J2,K2}} = quaternion:normal(Input);
	true ->
	    {R2,{I2,J2,K2}} = Input
    end,
    Den = math:sqrt(1.0 - R2*R2),
    Angle = 2.0*math:acos(R2),
    if
	Den < 0.001 ->
	    {{1.0,0.0,0.0},Angle};
	true ->
	    {{I2/Den,J2/Den,K2/Den},Angle}
    end.

toAxisAngleinDegrees({R,{I,J,K}}=Input)
  when is_float(I),is_float(J),is_float(K),is_float(R)->
    {Axis,Angle} = toAxisAngleinRadians(Input),
    {Axis,Angle / math:pi() * 180.0}.

toMatrix3({R,{I,J,K}})
   when is_float(I),is_float(J),is_float(K),is_float(R)->
    { 1.0 - 2.0*J*J - 2.0*K*K, 2.0*I*J - 2.0*R*K      , 2.0*I*K + 2.0*R*J,
      2.0*I*J + 2.0*R*K      , 1.0 - 2.0*I*I - 2.0*K*K, 2.0*J*K - 2.0*R*I,
      2.0*I*K - 2.0*R*J      , 2.0*J*K + 2.0*R*I      , 1.0 - 2.0*I*I - 2.0*J*J}.

normal(Q = {R,{I,J,K}})
   when is_float(R),is_float(I),is_float(J),is_float(K)->
    Mag = magnitude(Q),
    {R/Mag,{I/Mag,J/Mag,K/Mag}}.

magnitude({R,{I,J,K}})
   when is_float(R),is_float(I),is_float(J),is_float(K)->
    math:sqrt(R*R + I*I + J*J + K*K).

mul({R1,{I1,J1,K1}},{R2,{I2,J2,K2}})
   when is_float(R1),is_float(I1),is_float(J1),is_float(K1),
        is_float(R2),is_float(I2),is_float(J2),is_float(K2)->
    {R1*R2 - I1*I2 - J1*J2 - K1*K2,
     {R1*I2 + I1*R2 + J1*K2 - K1*J2,
      R1*J2 + J1*R2 + K1*I2 - I1*K2,
      R1*K2 + K1*R2 + I1*J2 - J1*I2}};
mul(S,{R,{I,J,K}}) 
  when is_float(S),is_float(R),is_float(I),is_float(J),is_float(K)->
    {R*S,{I*S,J*S,K*S}};
mul({R,{I,J,K}},S)
  when is_float(R),is_float(I),is_float(J),is_float(K),is_float(S)->
    {R*S,{I*S,J*S,K*S}}.

conjugate({R,{I,J,K}})
   when is_float(R),is_float(I),is_float(J),is_float(K)->
    {R,{-I,-J,-K}}.

add({R1,{I1,J1,K1}},{R2,{I2,J2,K2}})
   when is_float(R1),is_float(I1),is_float(J1),is_float(K1),
        is_float(R2),is_float(I2),is_float(J2),is_float(K2)->
    {R1 + R2,{I1 + I2,J1 + J2,K1 + K2}}.

