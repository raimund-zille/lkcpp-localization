#include "blobArray.h"
#include "blob.h"

Blob BlobArray::getClosestBlob() {
    Blob res;
    double minimalDistance = 10;
    for (Blob &b : blobs) {
        if (b.medianDistance < minimalDistance) {
            res = b;
            minimalDistance = res.medianDistance;
        }
    }
    return res; /// Check if set
}
