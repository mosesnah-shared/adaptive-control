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
    
    ckc1 = @( x ) ( isnumeric( x ) && x > 0      ) && ( length( x ) == 1 );                 % Size 
    ckc2 = @( x ) ( isstring( x ) || ischar( x ) );                                         % Style
    ckc3 = @( x ) ( isnumeric( x ) &&  all( x >= 0 & x <= 1 ) && ( length( x ) == 3 ) );    % Color
    ckc4 = @( x ) ( isstring( x ) ) || ( ischar( x ) );                                     % Name Check
    ckc5 = @( x ) ( isnumeric( x ) && x >= 0 && x <= 1 );                                   % Color alpha
                       
          
    addParameter( p,  'markerSize',                10, ckc1 );
    addParameter( p,   'lineWidth',                 5, ckc1 );
    
    addParameter( p, 'markerStyle',               'o', ckc2 );
    addParameter( p,   'lineStyle',               '-', ckc2 );    
    
    addParameter( p, 'markerColor',  0.82 * ones(1,3), ckc3 );
    addParameter( p,   'lineColor',  0.82 * ones(1,3), ckc3 );
    
    addParameter( p, 'markerAlpha',                 1, ckc5 );
    
    % For arrow (quiver) properties
    addParameter( p, 'maxHeadSize',               0.4, ckc1 );
    addParameter( p,  'arrowWidth',                 4, ckc1 );
    addParameter( p,  'arrowColor',  0.82 * ones(1,3), ckc3 );    
    
    addParameter( p,  'faceAlpha',                0.3, ckc5 );
    
    addParameter( p,        'name', "g" + num2str( randi ( 2^20 ) ), ckc4 );  
    
  
                                                                             
    parse( p, arguments{ : } )
    r = p.Results;

end

