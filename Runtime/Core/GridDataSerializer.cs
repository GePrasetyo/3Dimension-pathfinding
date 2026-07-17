using Cysharp.Threading.Tasks;
using System.IO;
using UnityEngine;
using UnityEngine.AddressableAssets;
using UnityEngine.ResourceManagement.AsyncOperations;

namespace Majinfwork.Pathfinding {
    /// <summary>
    /// Binary (de)serialization of baked grids and the Addressables loading path.
    /// Format v2: header (magic, version, dims, bake origin, point distance) + one
    /// staticInvalid bit per cell. Coords, world positions and neighbours are all
    /// derivable from the grid dimensions, so only the blocked flags are stored.
    /// </summary>
    public static class GridDataSerializer {
        private const float maxFrameTime = 0.01f;

        private const int formatMagic = 0x47524944; // "GRID"
        private const int formatVersion = 2;

        public static byte[] SerializeGrid(GridNode[][][] gridArray, GridCollection collection) {
            using(MemoryStream memoryStream = new()) {
                using(BinaryWriter writer = new(memoryStream)) {
                    writer.Write(formatMagic);
                    writer.Write(formatVersion);
                    writer.Write(collection.gridWidth);
                    writer.Write(collection.gridHeight);
                    writer.Write(collection.gridLength);
                    writer.Write(collection.startPoint.x);
                    writer.Write(collection.startPoint.y);
                    writer.Write(collection.startPoint.z);
                    writer.Write(collection.pointDistance);

                    int totalSize = collection.gridWidth * collection.gridHeight * collection.gridLength;
                    byte[] blockedBits = new byte[(totalSize + 7) / 8];

                    for(int x = 0; x < collection.gridWidth; x++) {
                        for(int y = 0; y < collection.gridHeight; y++) {
                            for(int z = 0; z < collection.gridLength; z++) {
                                if(gridArray[x][y][z].staticInvalid) {
                                    int index = collection.FlattenIndex(new Vector3Int(x, y, z));
                                    blockedBits[index >> 3] |= (byte)(1 << (index & 7));
                                }
                            }
                        }
                    }

                    writer.Write(blockedBits);
                }
                return memoryStream.ToArray();
            }
        }

        public static async UniTask<GridNode[][][]> LoadGridData(AssetReference dataFile, GridCollection collection) {
            AsyncOperationHandle<PathfindingGridData> handle = Addressables.LoadAssetAsync<PathfindingGridData>(dataFile);
            await handle.Task;
            if(handle.Status != AsyncOperationStatus.Succeeded) {
                Debug.LogError($"Failed to load grid data from {dataFile}");
                Addressables.Release(handle);
                return null;
            }
            await UniTask.NextFrame();
            GridNode[][][] gridData = await DeserializeGrid(handle.Result.byteData, collection);
            await UniTask.NextFrame();
            Addressables.Release(handle);
            return gridData;
        }

        public static async UniTask<GridNode[][][]> DeserializeGrid(byte[] data, GridCollection collection) {
            float taskTime = Time.realtimeSinceStartup;
            using(MemoryStream memoryStream = new(data)) {
                using(BinaryReader reader = new(memoryStream)) {
                    if(reader.ReadInt32() != formatMagic) {
                        Debug.LogError("[GridDataSerializer] Grid data uses an unknown format. Rebake it via the GridVolume inspector.");
                        return null;
                    }

                    int version = reader.ReadInt32();
                    if(version != formatVersion) {
                        Debug.LogError($"[GridDataSerializer] Grid data format version {version} does not match expected {formatVersion}. Rebake it via the GridVolume inspector.");
                        return null;
                    }

                    int width = reader.ReadInt32();
                    int height = reader.ReadInt32();
                    int length = reader.ReadInt32();

                    if(width != collection.gridWidth || height != collection.gridHeight || length != collection.gridLength) {
                        Debug.LogError($"[GridDataSerializer] Baked grid dimensions {width}x{height}x{length} do not match the GridVolume settings {collection.gridWidth}x{collection.gridHeight}x{collection.gridLength}. Rebake the grid.");
                        return null;
                    }

                    Vector3 bakedOrigin = new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
                    float bakedPointDistance = reader.ReadSingle();

                    if((bakedOrigin - collection.startPoint).sqrMagnitude > 0.0001f ||
                        Mathf.Abs(bakedPointDistance - collection.pointDistance) > 0.0001f) {
                        Debug.LogWarning($"[GridDataSerializer] Baked grid origin/spacing ({bakedOrigin}, {bakedPointDistance}) differs from the current GridVolume ({collection.startPoint}, {collection.pointDistance}). The blocked flags were baked at the old position and may be stale - rebake the grid.");
                    }

                    int totalSize = width * height * length;
                    byte[] blockedBits = reader.ReadBytes((totalSize + 7) / 8);

                    if(blockedBits.Length < (totalSize + 7) / 8) {
                        Debug.LogError("[GridDataSerializer] Grid data is truncated. Rebake it via the GridVolume inspector.");
                        return null;
                    }

                    GridNode[][][] gridArray = new GridNode[width][][];

                    for(int x = 0; x < width; x++) {
                        gridArray[x] = new GridNode[height][];
                        for(int y = 0; y < height; y++) {
                            gridArray[x][y] = new GridNode[length];
                            for(int z = 0; z < length; z++) {
                                if(Time.realtimeSinceStartup - taskTime > maxFrameTime) {
                                    await UniTask.NextFrame();
                                    taskTime = Time.realtimeSinceStartup;
                                }

                                GridNode grid = new();
                                grid.coords = new Vector3Int(x, y, z);
                                grid.worldPosition = collection.startPoint + new Vector3(x, y, z) * collection.pointDistance;

                                int index = collection.FlattenIndex(grid.coords);
                                grid.staticInvalid = (blockedBits[index >> 3] & (1 << (index & 7))) != 0;
                                gridArray[x][y][z] = grid;
                            }
                        }
                    }

                    return gridArray;
                }
            }
        }
    }
}
