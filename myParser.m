function r = myParser( arguments )
% % =============================================================== %
%   [DESCRIPTION]
%
%       myParser Function for simple parsing of the input variables
%
%
% % =============================================================== %
%   [CREATED BY]  Moses Nah
%   [EMAIL]       mosesnah@mit.edu
%   [AFFILIATION] Massachusetts Institute of Technology (MIT)
%   [MADE AT]     08-June-2020    
% % =============================================================== %

    p = inputParser( );

    arr2mat = @( x ) [ x( 1 ), x( 4 ), x( 5 ); ...
                       x( 4 ), x( 2 ), x( 6 ); ...
                       x( 5 ), x( 6 ), x( 3 )];                            % Changing a 1-by-6 vector to 3-by-3 symmetric matrix
    
    ckc1 = @( x ) ( isnumeric( x ) && all( x >= 0 )  ) && ( length( x ) == 2 ) ;
    ckc2 = @( x ) (  all( eig( arr2mat( x ) ) > 0 )  ) && ( size( x,1 ) == 2 && size( x,2 ) == 6 );
    
    addParameter( p,  'length'    ,  [ 0.294, 0.291 ], ckc1 );
    addParameter( p,  'lengthCOM' ,  [ 0.129, 0.112 ], ckc1 );
    addParameter( p,  'mass'      ,  [ 1.595, 0.869 ], ckc1 );
    addParameter( p,  'inertia'   ,  [ 0.011917 0.011937 0.001325, 0, 0, 0; 
                                       0.004765 0.004855 0.000472, 0, 0, 0], ckc2 );  % Check whether the matrix is positive definite. 
                                                                                      % The matrix is positive definite if.f the eigenvalues of the sym. part of the matrix are all postiive. 
    
    parse( p, arguments{ : } )
    r = p.Results;

end

