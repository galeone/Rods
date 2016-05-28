# Visual inspection of motorcycle connecting rods

Rods analyzes the images in the `rods` folder in order to extract:

. type
. baricenter
. orientation
. width
. height
. length
. width at the baricenter
. number of holes

For every hole:

. the center position
. the diameter

This is a classic computer vision problem. The peculiarity is how the problem of the touching rods has been solved.

Usually, a border following approach is used. In that case, I used the watershed transform (with markers).

Exploiting the knowledge about the domain, we can improve the markers and make them an approximation of the rod.

In such way the watershed algorithm performs better and the touching rods can easily speared and analyzed singularly.

The idea implementation starts at this line: https://github.com/galeone/Rods/blob/master/main.cpp#L217
