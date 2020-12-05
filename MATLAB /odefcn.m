function dx = odefcn( t, x, robot, M, C, G )
    % Function for doing a no-input 
    % Get the number of DOFs of the manipuluator
    
    
    q  = x(              1 : robot.nDOF );
    dq = x( robot.nDOF + 1 : end        );
        
    tmpM = double( subs( M,               robot.q,      q'   ) );
    tmpC = double( subs( C, [ robot.q, robot.dq ], [q', dq'] ) );
    tmpG = double( subs( G,               robot.q,      q'   ) );
    
    ddq = tmpM \ ( -tmpC * dq - tmpG' );
    dx = [ dq; ddq ];
end