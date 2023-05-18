
#include <iostream>
#include <limits>

#include "shapeprocessing.h" // Q) Why do we only need to include this header?

int main () {

  vector<Shape*> shape;



  //    * Allow the user to specify number of circles and rectangles

  //    * Create the shapes with random lengths to be capped to `MAX_LENGTH` - which is a const in    [shape_processing.h]./starter/library_test/shape_processing.h).

  //    * Allows user to enter a location x,y within (-MAX_SIZE, MAX_SIZE). `MAX_SIZE` is a const in [shape_processing.h]./starter/library_test/shape_processing.h) until all the shapes have been intersected


    //! What is this? Why is this a good practice?
    //! How has this been wrapped, look up namespace.
    std::cout << shapeprocessing::MAX_SIZE << std::endl;


    //! Additional questions
    //! Is shape and the shape_ in ShapeProcessing the same?
    //! If not, how do we change the code for this to occur?

    return 0;
}
