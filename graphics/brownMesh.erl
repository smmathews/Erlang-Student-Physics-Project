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

-module(brownMesh).
-export([load/1]).

load(FileName)->
    {ok,Bin} = file:read_file(FileName),
    << 
     4:32/little , 
     "IFS" , 
     0 , 
     1.0:32/little-float , 
     LenModelNameString:32/little , 
     _ModelNameString:LenModelNameString/binary , 
     9:32/little , 
     "VERTICES" , 
     0 , 
     NumVertices:32/little , 
     VertsAndOn/binary 
     >> = Bin,
    SizeVerts = 4*3*NumVertices,%4 bytes in a 32 bit float, 3 floats per vertex
    << 
     Vertices:SizeVerts/binary , 
     10:32/little , 
     "TRIANGLES" , 
     0 , 
     NumTriangles:32/little , 
     Indices/binary 
     >> = VertsAndOn,
    SizeIndices = 4*3*NumTriangles,%4 bytes in a 32 bit int, 3 int indices per triangle
    {
      {uniquePositions,SizeVerts,Vertices},
      {indices,SizeIndices,Indices}
     }.
