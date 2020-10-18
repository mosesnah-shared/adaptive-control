function JT = myTranspose( J )

[nR, nC] = size(J);

for i = 1 : nR
    for j = 1 : nC
        JT( j,i ) = J( i,j );
    end
end

end

