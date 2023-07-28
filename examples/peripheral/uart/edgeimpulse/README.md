To use a new edge impulse C++ library:

1. Save the debug_log.cpp, ei_classifier_porting.[cpp-h] files within the `edge-impulse-sdk/porting` directory.
2. Copy the new folders `edge-impulse-sdk`, `model-parameters`, `tflite-model` here.
3. Replace the files in `porting` directory from 1.
4. Then run the following command within the edge-impulse-sdk folder (arg-gcc doesn't support .cc extensions):

`find . -name '*.cc' -exec sh -c 'mv "$0" "${0%.cc}.cpp"' {} \;`

5. Run make command in `pca10040/blank/armgcc`