function dx = odefcn( t, x, M, C, G )
    % v = xr, qr, q, qe, each a 2-by-1 vector
    % Since we know q, we can calculate x, which is the actual end-effector position
    q1  = x( 1 );  q2  = x( 2 );
    dq1 = x( 3 );  dq2 = x( 4 );
    
    tmpM = double( subs( M, {'q1', 'q2' }, { q1, q2 }  ) );
    tmpC = double( subs( C, {'q1', 'q2', 'dq1', 'dq2' }, { q1, q2, dq1, dq2 } ) );
    tmpG = double( subs( G, {'q1', 'q2' }, { q1, q2 } ) );
    
    ddq = tmpM \ ( -tmpC * x(3:4) - tmpG' );
    dx = [dq1; dq2; ddq];
end