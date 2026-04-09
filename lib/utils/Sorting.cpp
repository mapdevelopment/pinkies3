#include <math.h>
#include <Sorting.h>

// Simple quicksort implementation
void quickSort(float arr[], int left, int right) {
    if (left >= right) return;

    float pivot = arr[right];
    int i = left - 1;

    for (int j = left; j < right; j++) {
        if (arr[j] <= pivot) {
            i++;
            float temp = arr[i];
            arr[i] = arr[j];
            arr[j] = temp;
        }
    }

    // Swap pivot into correct position
    float temp = arr[i + 1];
    arr[i + 1] = arr[right];
    arr[right] = temp;

    quickSort(arr, left, i);
    quickSort(arr, i + 2, right);
}

// Dominant cluster average
float get_dominant_cluster_average(const int N, float array[], const float tolerance) {
    if (N <= 0) return 0.0f;

    quickSort(array, 0, N - 1);  // Use our own quicksort

    float best_sum = 0.0f;
    int best_count = 0;

    float curr_sum = array[0];
    int curr_count = 1;

    for (int i = 1; i < N; i++) {
        if (fabsf(array[i] - array[i - 1]) < tolerance) {
            curr_sum += array[i];
            curr_count++;
        } else {
            if (curr_count > best_count && array[i - 1] > 0.0f) {
                best_sum = curr_sum;
                best_count = curr_count;
            }
            curr_sum = array[i];
            curr_count = 1;
        }
    }

    if (curr_count > best_count) {
        best_sum = curr_sum;
        best_count = curr_count;
    }

    return (best_count > 0) ? (best_sum / (best_count > 0 ? best_count : 1)) : 0.0f;
}
