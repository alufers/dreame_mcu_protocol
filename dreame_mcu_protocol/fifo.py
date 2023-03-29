import queue


class Fifo:
    q = queue.Queue()

    def write(self, data: bytes):
        for b in data:
            self.q.put(b)

    def read(self, size: int) -> bytes:
        return bytes([self.q.get() for _ in range(size)])
