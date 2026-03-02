function [vertices, faces] = create_cube_geometry(s)

% Half size
h = s/2;

vertices = [
    -h -h -h;
     h -h -h;
     h  h -h;
    -h  h -h;
    -h -h  h;
     h -h  h;
     h  h  h;
    -h  h  h];

faces = [
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8];

end