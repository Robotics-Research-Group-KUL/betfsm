import numpy as np
import threading
from collections import deque



class CircularNumpyBuffer:
    """
    A circular buffer with an underlying numpy array

    A circular buffer or ring-buffer remembers the last N additions, it has a start
    and an endpointer that wrap around at the end (hence ring/circular)

    Efficient additions and removals in front and at the end.

    This class is thread-safe and maintains a lock.
    """
    def __init__(self, capacity:int, cols:int, dtype=float):
        self.capacity = capacity
        self.cols     = cols
        self.data     = np.zeros((capacity, cols), dtype=dtype)
        self.start = 0           # index of the oldest element
        self.size  = 0           # number of valid elements
        self.end   = 0           # index where next element will be written
        self.lock = threading.Lock()

    def get_capacity(self):
        """Returns the capacity of this circular buffer, i.e. max size"""
        return self.capacity

    def add(self, row:int):
        """Add a new row, overwriting the oldest if full."""
        row = np.asarray(row)
        if row.shape != (self.cols,):
            raise ValueError(f"Row must have shape ({self.cols},)")
        with self.lock:
            self.data[self.end] = row
            self.end = (self.end + 1) % self.capacity
            if self.size < self.capacity:
                self.size += 1
            else:
                # buffer full, we overwrite oldest
                self.start = (self.start + 1) % self.capacity

    def consume_oldest(self):
        """Remove and return the oldest row."""
        with self.lock:
            if self.size == 0:
                raise IndexError("Buffer is empty")
            row = self.data[self.start].copy()
            self.start = (self.start + 1) % self.capacity
            self.size -= 1
            return row

    def consume_oldest_n(self, maxc:int):
        """ Consumes the maxc  oldest rows and returns them"""
        if maxc <= 0:
            return np.empty((0, self.cols), dtype=self.data.dtype)
        with self.lock:
            if self.size == 0:
                return np.empty((0, self.cols), dtype=self.data.dtype)
            n = min(maxc, self.size)
            start = self.start
            end = (self.start + n) % self.capacity
            if start < end:
                # contiguous
                out = self.data[start:end].copy()
            else:
                # wrap-around
                out = np.vstack((
                    self.data[start:],
                    self.data[:end]
                ))
            self.start = end
            self.size -= n
            return out

    def consume_newest(self):
        """Remove and return the newest row."""
        with self.lock:
            if self.size == 0:
                raise IndexError("Buffer is empty")
            newest_index = (self.end - 1) % self.capacity
            row = self.data[newest_index].copy()
            self.end = newest_index
            self.size -= 1
            return row

    def consume_newest_n(self, maxc:int):
        """Remove and return up to maxc newest rows."""
        if maxc <= 0:
            return np.empty((0, self.cols), dtype=self.data.dtype)
        with self.lock:
            if self.size == 0:
                return np.empty((0, self.cols), dtype=self.data.dtype)
            n = min(maxc, self.size)
            newest = (self.end - 1) % self.capacity
            oldest_needed = (newest - (n - 1)) % self.capacity
            # Extract rows
            if oldest_needed <= newest:
                out = self.data[oldest_needed:newest + 1].copy()
            else:
                out = np.vstack((
                    self.data[oldest_needed:],
                    self.data[:newest + 1]
                ))
            self.end = oldest_needed
            self.size -= n
            return out



    def peek_oldest(self):
        """peak for the oldest row (without removing)"""
        with self.lock:
            if self.size == 0:
                raise IndexError("Buffer is empty")

            return self.data[self.start].copy()

    def peek_oldest_n(self, maxc:int):
        """Return up to maxc oldest rows without removing them."""
        if maxc <= 0:
            return np.empty((0, self.cols), dtype=self.data.dtype)
        with self.lock:
            if self.size == 0:
                return np.empty((0, self.cols), dtype=self.data.dtype)
            n = min(maxc, self.size)
            start = self.start
            end = (self.start + n) % self.capacity
            if start < end:
                # contiguous
                return self.data[start:end].copy()
            else:
                # wrap-around
                return np.vstack((
                    self.data[start:],
                    self.data[:end]
                ))

    def peek_newest(self):
        """peak for the newest row (without removing)"""
        with self.lock:
            if self.size == 0:
                raise IndexError("Buffer is empty")
            newest_index = (self.end - 1) % self.capacity
            return self.data[newest_index].copy()

    def peek_newest_n(self, maxc:int):
        """ Get the maxc newest rows and return them (but not consume them)"""
        if maxc <= 0:
            return np.empty((0, self.cols), dtype=self.data.dtype)
        with self.lock:
            if self.size == 0:
                return np.empty((0, self.cols), dtype=self.data.dtype)
            n = min(maxc, self.size)
            newest = (self.end - 1) % self.capacity
            oldest_needed = (newest - (n - 1)) % self.capacity
            if oldest_needed <= newest:
                # contiguous
                return self.data[oldest_needed:newest + 1].copy()
            else:
                # wrap-around
                return np.vstack((
                    self.data[oldest_needed:],
                    self.data[:newest + 1]
                ))


    def to_array(self):
        """Return np.array representation of buffer"""
        with self.lock:
            if self.size == 0:
                return np.empty((0, self.cols), dtype=self.data.dtype)
            start = self.start
            end = (self.start + self.size) % self.capacity
            if start < end:
                # contiguous
                return self.data[start:end].copy()
            else:
                # wrap-around
                return np.vstack((
                    self.data[start:],
                    self.data[:end]
                ))
    def __len__(self):
        ### such that len(obj) returns the effective length
        with self.lock:
            return self.size


json_limit_circularbuffer_size = 200

def json_serializer(obj):
    global json_limit_circularbuffer_size
    """Fallback function for data types that standard json cannot serialize."""
    if isinstance(obj,CircularNumpyBuffer):
        if json_limit_circularbuffer_size>0:
            return list(obj.peek_newest_n(json_limit_circularbuffer_size))
        else:
            return list(obj.to_array())
    if isinstance(obj, deque):
        return list(obj)
    if isinstance(obj, np.ndarray):
        return obj.tolist()  # Converts matrices/arrays to nested lists
    if isinstance(obj, (np.integer, np.int64, np.int32)):
        return int(obj)
    if isinstance(obj, (np.floating, np.float64, np.float32)):
        return float(obj)
    
    raise TypeError(f"Type {type(obj)} not serializable")

