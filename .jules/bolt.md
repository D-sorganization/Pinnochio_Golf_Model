## 2025-12-10 - [Recursive Lookup Bottleneck in Exporters]
**Learning:** The `MJCFExporter` used a recursive function that iterated over the entire segment list `self.spec["segments"]` to find children of the current node. This resulted in O(N^2) complexity, causing significant slowdowns for models with many segments (>1000).
**Action:** When working with hierarchical data stored in flat lists, always pre-process the list into an adjacency map (`parent -> children`) to allow O(1) lookups during recursion, reducing overall complexity to O(N).
