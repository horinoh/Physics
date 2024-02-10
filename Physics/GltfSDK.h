#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string_view>
#include <cmath>

#include <GLTFSDK/GLTF.h>
#include <GLTFSDK/GLTFResourceReader.h>
#include <GLTFSDK/GLBResourceReader.h>
#include <GLTFSDK/Deserialize.h>

namespace Gltf {
	static bool IsBinary(std::string_view Path) {
		std::ifstream In(data(Path), std::ios::in | std::ios::binary);
		if (!In.fail()) {
			In.seekg(0, std::ios_base::end);
			const auto Size = In.tellg();
			if (Size) {
				In.seekg(0, std::ios_base::beg);
				std::vector<std::byte> Buf(Size);
				In.read(reinterpret_cast<char*>(data(Buf)), 4);
				return std::string("glTF") == std::string(reinterpret_cast<const char*>(data(Buf)));
			}
		}
		return false;
	}

    class StreamReader : public Microsoft::glTF::IStreamReader
    {
    public:
        virtual std::shared_ptr<std::istream> GetInputStream(const std::string& FilePath) const override {
            return std::make_shared<std::ifstream>(FilePath, std::ios_base::binary);
        }
    };
	class SDK
	{	
	protected:
		std::unique_ptr<Microsoft::glTF::GLTFResourceReader> ResourceReader;
		Microsoft::glTF::Document Document;
	public:
		virtual void Process(const Microsoft::glTF::Buffer& Buffer) {
			std::cout << "\tByteLength = " << Buffer.byteLength << std::endl;
			std::cout << "\tUri = " << Buffer.uri.substr(0, 32) << " ..." << std::endl;
		}
		virtual void Process(const Microsoft::glTF::BufferView& BufferView) {
			std::cout << "BufferView [" << BufferView.id << "] : " << BufferView.name << std::endl;
			std::cout << "\tBufferViewTarget = ";
			switch (BufferView.target)
			{
			case Microsoft::glTF::BufferViewTarget::UNKNOWN_BUFFER:
				std::cout << "UNKNOWN_BUFFER";
				break;
			case Microsoft::glTF::BufferViewTarget::ARRAY_BUFFER: 
				std::cout << "ARRAY_BUFFER";
				break;
			case Microsoft::glTF::BufferViewTarget::ELEMENT_ARRAY_BUFFER: 
				std::cout << "ELEMENT_ARRAY_BUFFER";
				break;
			default:
				break;
			}
			std::cout << std::endl;

			std::cout << "\tByteOffset = " << BufferView.byteOffset << std::endl;
			std::cout << "\tByteLength = " << BufferView.byteLength << std::endl;
			std::cout << "\tByteStride = " << BufferView.byteStride << std::endl;

			if (Document.buffers.Has(BufferView.bufferId)) {
				Process(Document.buffers.Get(BufferView.bufferId));
			}
		}
		virtual void Process(const Microsoft::glTF::Accessor& Accessor, std::string_view name = "") {
			std::cout << "Accessor [" << Accessor.id << "] : " << (empty(Accessor.name) ? name : Accessor.name) << std::endl;

			std::cout << "\tComponentType = " << Microsoft::glTF::Accessor::GetComponentTypeName(Accessor.componentType) << std::endl;
			switch (Accessor.componentType)
			{
			case Microsoft::glTF::ComponentType::COMPONENT_UNKNOWN: break;
			case Microsoft::glTF::ComponentType::COMPONENT_BYTE:break;
			case Microsoft::glTF::ComponentType::COMPONENT_UNSIGNED_BYTE:break;
			case Microsoft::glTF::ComponentType::COMPONENT_SHORT:break;
			case Microsoft::glTF::ComponentType::COMPONENT_UNSIGNED_SHORT:break;
			case Microsoft::glTF::ComponentType::COMPONENT_UNSIGNED_INT:break;
			case Microsoft::glTF::ComponentType::COMPONENT_FLOAT:break;
			default: break;
			}

			std::cout << "\tAccessorType = " << Microsoft::glTF::Accessor::GetAccessorTypeName(Accessor.type) << std::endl;
			switch (Accessor.type)
			{
			case Microsoft::glTF::AccessorType::TYPE_UNKNOWN: break;
			case Microsoft::glTF::AccessorType::TYPE_SCALAR: break;
			case Microsoft::glTF::AccessorType::TYPE_VEC2: break;
			case Microsoft::glTF::AccessorType::TYPE_VEC3: break;
			case Microsoft::glTF::AccessorType::TYPE_VEC4: break;
			case Microsoft::glTF::AccessorType::TYPE_MAT2: break;
			case Microsoft::glTF::AccessorType::TYPE_MAT3: break;
			case Microsoft::glTF::AccessorType::TYPE_MAT4: break;
			default: break;
			}

			//!< COMPONENT_FLOAT, TYPE_VEC3 の場合なら float3 の個数
			std::cout << "\tCount = " << Accessor.count << std::endl;
			std::cout << "\tByteOffset = " << Accessor.byteOffset << std::endl;
			std::cout << "\tNormalized = " << (Accessor.normalized ? "true" : "false") << std::endl;

			if (Document.bufferViews.Has(Accessor.bufferViewId)) {
				Process(Document.bufferViews.Get(Accessor.bufferViewId));
			}
		}
		virtual void Process(const Microsoft::glTF::MeshPrimitive& Primitive) {
			std::cout << "\tMeshMode = ";
			switch (Primitive.mode)
			{
			case Microsoft::glTF::MeshMode::MESH_POINTS:
				std::cout << "MESH_POINTS";
				break;
			case Microsoft::glTF::MeshMode::MESH_LINES:
				std::cout << "MESH_LINES";
				break;
			case Microsoft::glTF::MeshMode::MESH_LINE_LOOP:
				std::cout << "MESH_LINE_LOOP";
				break;
			case Microsoft::glTF::MeshMode::MESH_LINE_STRIP:
				std::cout << "MESH_LINE_STRIP";
				break;
			case Microsoft::glTF::MeshMode::MESH_TRIANGLES:
				std::cout << "MESH_TRIANGLES";
				break;
			case Microsoft::glTF::MeshMode::MESH_TRIANGLE_STRIP:
				std::cout << "MESH_TRIANGLE_STRIP";
				break;
			case Microsoft::glTF::MeshMode::MESH_TRIANGLE_FAN:
				std::cout << "MESH_TRIANGLE_FAN";
				break;
			default:
				break;
			}
			std::cout << std::endl;

			//!< インデックス
			if(Document.accessors.Has(Primitive.indicesAccessorId)) {
				const auto& Accessor = Document.accessors.Get(Primitive.indicesAccessorId);
				Process(Accessor);

				switch (Accessor.componentType)
				{
				case Microsoft::glTF::ComponentType::COMPONENT_UNSIGNED_SHORT:
					switch (Accessor.type)
					{
					case Microsoft::glTF::AccessorType::TYPE_SCALAR:
						//!< 埋め込まれていないとこの取り方はできない？
						//{
						//	const auto Data = ResourceReader->ReadBinaryData<uint16_t>(Document, Accessor);
						//	assert(Accessor.count == size(Data) && "");
						//}
						break;
					default: break;
					}
					break;
				case Microsoft::glTF::ComponentType::COMPONENT_UNSIGNED_INT:
					switch (Accessor.type)
					{
					case Microsoft::glTF::AccessorType::TYPE_SCALAR: 
						//!< 埋め込まれていないとこの取り方はできない？
						//{
						//	const auto Data = ResourceReader->ReadBinaryData<uint32_t>(Document, Accessor);
						//	assert(Accessor.count == size(Data) && "");
						//}
						break;
					default: break;
					}
					break;
				default: break;
				}

				//!< UNKNOWN_BUFFER が返る事が普通にあるので assert しない
				//if (Document.bufferViews.Has(Accessor.bufferViewId)) { assert(Microsoft::glTF::BufferViewTarget::ELEMENT_ARRAY_BUFFER == Document.bufferViews.Get(Accessor.bufferViewId).target && ""); }
			}

			//!< バーテックス
			if (size(Primitive.attributes)) {
				for (const auto& i : Primitive.attributes) {
					Process(Document.accessors.Get(i.second), i.first);
				}
			}
			if (size(Primitive.targets)) {
				std::cout << "\tMorphTarget" << std::endl;
				for (const auto& i : Primitive.targets) {
					if (Document.accessors.Has(i.positionsAccessorId)) {
						const auto& Accessor = Document.accessors.Get(i.positionsAccessorId);
						std::cout << "\t\tPositions [" << Accessor.id << "] : " << Accessor.name << std::endl;
					}
					if (Document.accessors.Has(i.normalsAccessorId)) {
						const auto& Accessor = Document.accessors.Get(i.normalsAccessorId);
						std::cout << "\t\tNormals [" << Accessor.id << "] : " << Accessor.name << std::endl;
					}
					if (Document.accessors.Has(i.tangentsAccessorId)) {
						const auto& Accessor = Document.accessors.Get(i.tangentsAccessorId);
						std::cout << "\t\tTangents [" << Accessor.id << "] : " << Accessor.name << std::endl;
					}
				}
			}
		}
		virtual void Process(const Microsoft::glTF::Mesh& Mesh) {
			std::cout << "Mesh [" << Mesh.id << "] : " << Mesh.name << std::endl;

			if (!empty(Mesh.weights)) {
				std::cout << "\tWeights = (";
				for (auto i : Mesh.weights) {
					std::cout << i << ", ";
				}
				std::cout << ")" << std::endl;
			}

			for (const auto& i : Mesh.primitives) {
				Process(i);
			}
		}
		virtual void Process(const Microsoft::glTF::Node& Node) {
			std::cout << "Node [" << Node.id << "] : " << Node.name << std::endl;

			switch (Node.GetTransformationType()) {
			case Microsoft::glTF::TRANSFORMATION_IDENTITY:
				break;
			case Microsoft::glTF::TRANSFORMATION_MATRIX:
				std::cout << "\tM = (" << Node.matrix.values[ 0] << ", " << Node.matrix.values[ 1] << ", " << Node.matrix.values[ 2] << ", " << Node.matrix.values[ 3] << ")" << std::endl;
				std::cout << "\t    (" << Node.matrix.values[ 4] << ", " << Node.matrix.values[ 5] << ", " << Node.matrix.values[ 6] << ", " << Node.matrix.values[ 7] << ")" << std::endl;
				std::cout << "\t    (" << Node.matrix.values[ 8] << ", " << Node.matrix.values[ 9] << ", " << Node.matrix.values[10] << ", " << Node.matrix.values[11] << ")" << std::endl;
				std::cout << "\t    (" << Node.matrix.values[12] << ", " << Node.matrix.values[13] << ", " << Node.matrix.values[14] << ", " << Node.matrix.values[15] << ")" << std::endl;
				break;
			case Microsoft::glTF::TRANSFORMATION_TRS:
				std::cout << "\tT = (" << Node.translation.x << ", " << Node.translation.y << ", " << Node.translation.z << ")" << std::endl;
				std::cout << "\tR = (" << Node.rotation.x << ", " << Node.rotation.y << ", " << Node.rotation.z << ", " << Node.rotation.w << ")" << std::endl;
				std::cout << "\tS = (" << Node.scale.x << ", " << Node.scale.y << ", " << Node.scale.z << ")" << std::endl;
				break;
			default: break;
			} 

			if (Document.meshes.Has(Node.meshId)) {
				Process(Document.meshes.Get(Node.meshId));
			}

			if (!empty(Node.weights)) {
				std::cout << "\tWeights = (";
					for (auto i : Node.weights) {
						std::cout << i << ", ";
					}
				std::cout << ")" << std::endl;
			}

			for (auto i : Node.children) {
				Process(Document.nodes.Get(i));
			}
		}
		virtual void Process(const Microsoft::glTF::Scene& Scene) {
			std::cout << "Scene [" << Scene.id << "] : " << Scene.name << std::endl;

			for (auto i : Scene.nodes) {
				Process(Document.nodes.Get(i));
			}
		}
		virtual void ProcessScene() {
			if (Document.HasDefaultScene()) {
				//!< デフォルトシーンがある場合
				Process(Document.scenes.Get(Document.GetDefaultScene().id));
			}
			else
			{
				//!< デフォルトシーンが無い場合は全シーン
				for (const auto& i : Document.scenes.Elements()) {
					Process(i);
				}
			}
		}
		//virtual void ProcessAnimation() {
		//	for (const auto& i : Document.animations.Elements()) {
		//		Process(i);
		//	}
		//}
		virtual void Process() {
			std::cout << "Version:    " << Document.asset.version << std::endl;
			std::cout << "MinVersion: " << Document.asset.minVersion << std::endl;
			std::cout << "Generator:  " << Document.asset.generator << std::endl;
			std::cout << "Copyright:  " << Document.asset.copyright << std::endl;

			std::cout << "Scene Count: " << Document.scenes.Size() << std::endl;
			if (Document.scenes.Size() > 0U)
			{
				std::cout << "Default Scene Index: " << Document.GetDefaultScene().id << std::endl;
			}

			std::cout << "Node Count:     " << Document.nodes.Size() << std::endl;
			std::cout << "Camera Count:   " << Document.cameras.Size() << std::endl;
			std::cout << "Material Count: " << Document.materials.Size() << std::endl;

			std::cout << "Mesh Count: " << Document.meshes.Size() << std::endl;
			std::cout << "Skin Count: " << Document.skins.Size() << std::endl;

			std::cout << "Image Count:   " << Document.images.Size() << std::endl;
			std::cout << "Texture Count: " << Document.textures.Size() << std::endl;
			std::cout << "Sampler Count: " << Document.samplers.Size() << std::endl;

			std::cout << "Buffer Count:     " << Document.buffers.Size() << std::endl;
			std::cout << "BufferView Count: " << Document.bufferViews.Size() << std::endl;
			std::cout << "Accessor Count:   " << Document.accessors.Size() << std::endl;

			std::cout << "Animation Count: " << Document.animations.Size() << std::endl;

			//!< シーン
			ProcessScene();
		}

		//!< リンカエラー 4099 が出る(#pragma では回避できない)ので以下のようにしている 
		//!< Configuration Properties - Linker - CommandLine - AdditionalOptions - /ignore:4099
		void Load(const std::filesystem::path& Path) {
			auto Reader = std::make_unique<StreamReader>();
			auto In = Reader->GetInputStream(Path.string());

			if (std::string(".") + Microsoft::glTF::GLTF_EXTENSION == Path.extension()) {
				auto GLTFReader = std::make_unique<Microsoft::glTF::GLTFResourceReader>(std::move(Reader));

				//!< std::stringstream を使用して、string へ GLTF ファイルを読み込む
				std::stringstream SS;
				SS << In->rdbuf();
				Document = Microsoft::glTF::Deserialize(SS.str());

				ResourceReader = std::move(GLTFReader);
			}
			if (std::string(".") + Microsoft::glTF::GLB_EXTENSION == Path.extension()) {
				auto GLBReader = std::make_unique<Microsoft::glTF::GLBResourceReader>(std::move(Reader), std::move(In));

				Document = Microsoft::glTF::Deserialize(GLBReader->GetJson());

				ResourceReader = std::move(GLBReader);
			}

			Process();
		}
	};
}
