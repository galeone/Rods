# Visual inspection of motorcycle connecting rods

Rods analyzes the images in the `rods` folder in order to extract:

1. type
2. baricenter
3. orientation
4. width
5. height
6. length
7. width at the baricenter
8. number of holes

For every hole:

1. the center position
2. the diameter

This is a classic computer vision problem. The peculiarity is how the problem of the touching rods has been solved.

Usually, a border following approach is used. In that case, I used the watershed transform (with markers).

Exploiting the knowledge about the domain, we can improve the markers and make them an approximation of the rod.

In such way the watershed algorithm performs better and the touching rods can easily speared and analyzed singularly.

The idea implementation starts at this line: https://github.com/galeone/Rods/blob/master/main.cpp#L217
