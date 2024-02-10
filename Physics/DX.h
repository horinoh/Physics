#pragma once

#include <Windows.h>

#include <vector>
#include <array>
#include <cassert>
#include <iterator>
#include <numeric>
#include <bitset>
#include <algorithm>
#include <format>
#include <fstream>
#include <thread>

#include <d3d12.h>
#include <DXGI1_6.h>
#include <DirectXColors.h>
#include <d3dcompiler.h>

#include <winrt/base.h>
#define COM_PTR winrt::com_ptr
#define COM_PTR_GET(_x) _x.get()
#define COM_PTR_PUT(_x) _x.put()
#define COM_PTR_PUTVOID(_x) _x.put_void()
#define COM_PTR_UUIDOF_PUTVOID(_x) __uuidof(_x), COM_PTR_PUTVOID(_x)
#define COM_PTR_RESET(_x) _x = nullptr
#define COM_PTR_AS(_x, _y) winrt::copy_to_abi(_x, *_y.put_void());
#define COM_PTR_COPY(_x, _y) _x.copy_from(COM_PTR_GET(_y))

#define SHADER_ROOT_ACCESS_DENY_ALL (D3D12_ROOT_SIGNATURE_FLAG_DENY_VERTEX_SHADER_ROOT_ACCESS | D3D12_ROOT_SIGNATURE_FLAG_DENY_HULL_SHADER_ROOT_ACCESS | D3D12_ROOT_SIGNATURE_FLAG_DENY_DOMAIN_SHADER_ROOT_ACCESS | D3D12_ROOT_SIGNATURE_FLAG_DENY_GEOMETRY_SHADER_ROOT_ACCESS | D3D12_ROOT_SIGNATURE_FLAG_DENY_PIXEL_SHADER_ROOT_ACCESS | D3D12_ROOT_SIGNATURE_FLAG_DENY_AMPLIFICATION_SHADER_ROOT_ACCESS | D3D12_ROOT_SIGNATURE_FLAG_DENY_MESH_SHADER_ROOT_ACCESS)
#define SHADER_ROOT_ACCESS_VS (SHADER_ROOT_ACCESS_DENY_ALL & ~D3D12_ROOT_SIGNATURE_FLAG_DENY_VERTEX_SHADER_ROOT_ACCESS)
#define SHADER_ROOT_ACCESS_GS (SHADER_ROOT_ACCESS_DENY_ALL & ~D3D12_ROOT_SIGNATURE_FLAG_DENY_GEOMETRY_SHADER_ROOT_ACCESS)
#define SHADER_ROOT_ACCESS_VS_GS (SHADER_ROOT_ACCESS_DENY_ALL & ~(D3D12_ROOT_SIGNATURE_FLAG_DENY_VERTEX_SHADER_ROOT_ACCESS | D3D12_ROOT_SIGNATURE_FLAG_DENY_GEOMETRY_SHADER_ROOT_ACCESS))
#define SHADER_ROOT_ACCESS_PS (SHADER_ROOT_ACCESS_DENY_ALL & ~D3D12_ROOT_SIGNATURE_FLAG_DENY_PIXEL_SHADER_ROOT_ACCESS)

#include "Common.h"

#ifdef _DEBUG
#define VERIFY_SUCCEEDED(x) { const auto HR = (x); if(!SUCCEEDED(HR)){ LOG(data(std::format("HRESULT = {:#x}\n", static_cast<UINT>(HR)))); __debugbreak(); } }
#define VERIFY(x) if(!(x)){ __debugbreak(); }
#else
#define VERIFY_SUCCEEDED(x) (x)
#define VERIFY(x) (x)
#endif

class DX : public Win
{
public:
	virtual void OnCreate(HWND hWnd, HINSTANCE hInstance, LPCWSTR Title) {
		GetClientRect(hWnd, &Rect);
		const auto W = Rect.right - Rect.left, H = Rect.bottom - Rect.top;
		LOG(data(std::format("Rect = {} x {}\n", W, H)));

		CreateDevice(hWnd);
		CreateCommandQueue();
		CreateFence();
		CreateSwapChain(hWnd, DXGI_FORMAT_R8G8B8A8_UNORM);
		CreateCommandList();
		CreateGeometry();
		CreateConstantBuffer();
		CreateTexture();
		CreateStaticSampler();
		CreateRootSignature();
		CreatePipelineState();
		CreateDescriptor();

		OnExitSizeMove(hWnd, hInstance);
	}
	virtual void OnExitSizeMove(HWND hWnd, HINSTANCE hInstance) {
#define TIMER_ID 1000 //!< 何でも良い
		KillTimer(hWnd, TIMER_ID);
		{
			OnPreDestroy();

			GetClientRect(hWnd, &Rect);

			const auto W = Rect.right - Rect.left, H = Rect.bottom - Rect.top;
			CreateViewport(static_cast<const FLOAT>(W), static_cast<const FLOAT>(H));

			for (auto i = 0; i < size(DirectCommandLists); ++i) {
				PopulateBundleCommandList(i);
				PopulateCommandList(i);
			}
		}
		SetTimer(hWnd, TIMER_ID, 1000 / 60, nullptr);
	}
	virtual void OnTimer(HWND hWnd, HINSTANCE hInstance) { SendMessage(hWnd, WM_PAINT, 0, 0); }
	virtual void OnPaint(HWND hWnd, HINSTANCE hInstance)  { Draw(); }
	//!< 解放前に、終了を待たなくてはならないものをここで待つ
	virtual void OnPreDestroy() {
		WaitForFence(COM_PTR_GET(GraphicsCommandQueue), COM_PTR_GET(GraphicsFence));
	}
	virtual void OnDestroy(HWND hWnd, HINSTANCE hInstance)  { }

public:
	virtual void CreateDevice([[maybe_unused]] HWND hWnd);
	virtual void CreateCommandQueue();
	virtual void CreateFence() {
		VERIFY_SUCCEEDED(Device->CreateFence(0, D3D12_FENCE_FLAG_NONE, COM_PTR_UUIDOF_PUTVOID(GraphicsFence)));
	}
	
	virtual void CreateSwapChain(HWND hWnd, const DXGI_FORMAT ColorFormat, const UINT Width, const UINT Height);
	virtual void GetSwapChainResource();
	virtual void CreateSwapChain(HWND hWnd, const DXGI_FORMAT ColorFormat) {
		CreateSwapChain(hWnd, ColorFormat, Rect.right - Rect.left, Rect.bottom - Rect.top);
		GetSwapChainResource();
	}

	void CreateDirectCommandList(const size_t Num);
	virtual void CreateDirectCommandList() { CreateDirectCommandList(size(SwapChainBackBuffers)); }
	void CreateBundleCommandList(const size_t Num);
	virtual void CreateBundleCommandList() { CreateBundleCommandList(size(SwapChainBackBuffers)); }
	virtual void CreateCommandList() {
		CreateDirectCommandList();
		CreateBundleCommandList();
	}

	virtual void CreateGeometry() {}
	virtual void CreateConstantBuffer() {}
	virtual void CreateTexture() {}
	virtual void CreateStaticSampler() {}
	
	template<typename T = D3D12_ROOT_PARAMETER1> void SerializeRootSignature(COM_PTR<ID3DBlob>& Blob, const std::vector<T>& RPs, const std::vector<D3D12_STATIC_SAMPLER_DESC>& SSDs, const D3D12_ROOT_SIGNATURE_FLAGS Flags);
	virtual void CreateRootSignature() {}

	static void CreatePipelineStateVsPsDsHsGs(COM_PTR<ID3D12PipelineState>& PST,
		ID3D12Device* Device, ID3D12RootSignature* RS,
		const D3D12_PRIMITIVE_TOPOLOGY_TYPE PTT,
		const std::vector<D3D12_RENDER_TARGET_BLEND_DESC>& RTBDs,
		const D3D12_RASTERIZER_DESC& RD,
		const D3D12_DEPTH_STENCIL_DESC& DSD,
		const D3D12_SHADER_BYTECODE VS, const D3D12_SHADER_BYTECODE PS, const D3D12_SHADER_BYTECODE DS, const D3D12_SHADER_BYTECODE HS, const D3D12_SHADER_BYTECODE GS,
		const std::vector<D3D12_INPUT_ELEMENT_DESC>& IEDs,
		const std::vector<DXGI_FORMAT>& RTVFormats,
		LPCWSTR Name = nullptr);
	virtual void CreatePipelineState() {}
	
	virtual void CreateDescriptor() {}

	virtual void CreateViewport(const FLOAT Width, const FLOAT Height, const FLOAT MinDepth = 0.0f, const FLOAT MaxDepth = 1.0f);
	virtual void PopulateBundleCommandList([[maybe_unused]] const size_t i) {}
	virtual void PopulateCommandList(const size_t i) {
		PopulateCommandList_Clear(i, DirectX::Colors::SkyBlue);
	}

	static void WaitForFence(ID3D12CommandQueue* CQ, ID3D12Fence* Fence);
	virtual void DrawFrame([[maybe_unused]] const UINT i) {}
	virtual void SubmitGraphics(const UINT i);
	virtual void Present();
	virtual void Draw();

protected:
	static void CreateBufferResource(ID3D12Resource** Resource, ID3D12Device* Device, const size_t Size, const D3D12_RESOURCE_FLAGS RF, const D3D12_HEAP_TYPE HT, const D3D12_RESOURCE_STATES RS, const void* Source = nullptr) {
		const D3D12_RESOURCE_DESC RD = {
			.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
			.Alignment = 0,
			.Width = Size, .Height = 1, 
			.DepthOrArraySize = 1, .MipLevels = 1,
			.Format = DXGI_FORMAT_UNKNOWN,
			.SampleDesc = DXGI_SAMPLE_DESC({.Count = 1, .Quality = 0 }),
			.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
			.Flags = RF
		};
		const D3D12_HEAP_PROPERTIES HP = {
			.Type = HT,
			.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
			.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
			.CreationNodeMask = 0, 
			.VisibleNodeMask = 0 
		};
		VERIFY_SUCCEEDED(Device->CreateCommittedResource(&HP, D3D12_HEAP_FLAG_NONE, &RD, RS, nullptr, IID_PPV_ARGS(Resource)));
		if (nullptr != Source) {
			BYTE* Data;
			VERIFY_SUCCEEDED((*Resource)->Map(0, nullptr, reinterpret_cast<void**>(&Data))); {
				memcpy(Data, Source, Size);
			} (*Resource)->Unmap(0, nullptr);
		}
	}
	static void CreateBufferResource(ID3D12Resource** Resource, ID3D12Device* Device, const std::vector<D3D12_SUBRESOURCE_DATA>& SRDs, const std::vector<D3D12_PLACED_SUBRESOURCE_FOOTPRINT>& PSFs, const std::vector<UINT>& NumRows, const std::vector<UINT64>& RowSizeInBytes, const UINT64 TotalBytes) {
		const D3D12_RESOURCE_DESC RD = {
			.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
			.Alignment = 0,
			.Width = TotalBytes, .Height = 1,
			.DepthOrArraySize = 1, .MipLevels = 1,
			.Format = DXGI_FORMAT_UNKNOWN,
			.SampleDesc = DXGI_SAMPLE_DESC({.Count = 1, .Quality = 0 }),
			.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
			.Flags = D3D12_RESOURCE_FLAG_NONE
		};
		constexpr D3D12_HEAP_PROPERTIES HP = {
			.Type = D3D12_HEAP_TYPE_UPLOAD,
			.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
			.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
			.CreationNodeMask = 0, .VisibleNodeMask = 0
		};
		VERIFY_SUCCEEDED(Device->CreateCommittedResource(&HP, D3D12_HEAP_FLAG_NONE, &RD, D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(Resource)));

		BYTE* Data;
		VERIFY_SUCCEEDED((*Resource)->Map(0, nullptr, reinterpret_cast<void**>(&Data))); {
			for (auto i = 0; i < size(PSFs); ++i) {
				const auto NR = NumRows[i];
				const auto RSIB = RowSizeInBytes[i];
				const D3D12_MEMCPY_DEST MCD = {
					.pData = Data + PSFs[i].Offset,
					.RowPitch = PSFs[i].Footprint.RowPitch,
					.SlicePitch = static_cast<SIZE_T>(PSFs[i].Footprint.RowPitch) * NR
				};
				const auto& SRD = SRDs[i];
				for (UINT j = 0; j < PSFs[i].Footprint.Depth; ++j) {
					auto Dst = reinterpret_cast<BYTE*>(MCD.pData) + MCD.SlicePitch * j;
					const auto Src = reinterpret_cast<const BYTE*>(SRD.pData) + SRD.SlicePitch * j;
					for (UINT k = 0; k < NR; ++k) {
						memcpy(Dst + MCD.RowPitch * k, Src + SRD.RowPitch * k, RSIB);
					}
				}
			}
		} (*Resource)->Unmap(0, nullptr);
	}
	static void CreateTextureResource(ID3D12Resource** Resource, ID3D12Device* Device, const UINT64 Width, const UINT Height, const UINT16 DepthOrArraySize, const UINT16 MipLevels, DXGI_FORMAT Format, const D3D12_RESOURCE_FLAGS RF, const D3D12_RESOURCE_STATES RS) {
		constexpr D3D12_HEAP_PROPERTIES HP = {
			.Type = D3D12_HEAP_TYPE_DEFAULT,
			.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
			.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
			.CreationNodeMask = 0, .VisibleNodeMask = 0
		};
		const D3D12_RESOURCE_DESC RD = {
			.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D,
			.Alignment = 0,
			.Width = Width, .Height = Height,
			.DepthOrArraySize = DepthOrArraySize,
			.MipLevels = MipLevels,
			.Format = Format,
			.SampleDesc = DXGI_SAMPLE_DESC({.Count = 1, .Quality = 0 }),
			.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN,
			.Flags = RF
		};
		assert(!(RD.Flags & (D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET | D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL)) && "非 RENDER_TARGET, DEPTH_STENCIL の場合、pOptimizedClearValue を使用しない");
		VERIFY_SUCCEEDED(Device->CreateCommittedResource(&HP, D3D12_HEAP_FLAG_NONE, &RD, RS, nullptr, IID_PPV_ARGS(Resource)));
	}
	static void CreateRenderTextureResource(ID3D12Resource** Resource, ID3D12Device* Device, const UINT64 Width, const UINT Height, const UINT16 DepthOrArraySize, const UINT16 MipLevels, const D3D12_CLEAR_VALUE& CV, const D3D12_RESOURCE_FLAGS RF, const D3D12_RESOURCE_STATES RS) {
		constexpr D3D12_HEAP_PROPERTIES HP = {
			.Type = D3D12_HEAP_TYPE_DEFAULT,
			.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
			.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
			.CreationNodeMask = 0, .VisibleNodeMask = 0
		};
		const D3D12_RESOURCE_DESC RD = {
			.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D,
			.Alignment = 0,
			.Width = Width, .Height = Height,
			.DepthOrArraySize = DepthOrArraySize,
			.MipLevels = MipLevels,
			.Format = CV.Format,
			.SampleDesc = DXGI_SAMPLE_DESC({.Count = 1, .Quality = 0 }),
			.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN,
			.Flags = RF
		};
		VERIFY_SUCCEEDED(Device->CreateCommittedResource(&HP, D3D12_HEAP_FLAG_NONE, &RD, RS, &CV, IID_PPV_ARGS(Resource)));
	}
	static void ExecuteAndWait(ID3D12CommandQueue* CQ, ID3D12CommandList* CL, ID3D12Fence* Fence) {
		const std::array CLs = { CL };
		CQ->ExecuteCommandLists(static_cast<UINT>(size(CLs)), data(CLs));
		WaitForFence(CQ, Fence);
	}
	static void ResourceBarrier(ID3D12GraphicsCommandList* GCL, 
		ID3D12Resource* Resource, const D3D12_RESOURCE_STATES Before, const D3D12_RESOURCE_STATES After) {
		const std::array RBs = {
			D3D12_RESOURCE_BARRIER({
				.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION,
				.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE,
				.Transition = D3D12_RESOURCE_TRANSITION_BARRIER({
					.pResource = Resource,
					.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES,
					.StateBefore = Before, .StateAfter = After
				})
			})
		};
		GCL->ResourceBarrier(static_cast<UINT>(size(RBs)), data(RBs));
	}
	static void ResourceBarrier2(ID3D12GraphicsCommandList* GCL, 
		ID3D12Resource* Resource0, const D3D12_RESOURCE_STATES Before0, const D3D12_RESOURCE_STATES After0,
		ID3D12Resource* Resource1, const D3D12_RESOURCE_STATES Before1, const D3D12_RESOURCE_STATES After1) {
		const std::array RBs = {
			D3D12_RESOURCE_BARRIER({
				.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION,
				.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE,
				.Transition = D3D12_RESOURCE_TRANSITION_BARRIER({
					.pResource = Resource0,
					.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES,
					.StateBefore = Before0, .StateAfter = After0
				})
			}),
			D3D12_RESOURCE_BARRIER({
				.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION,
				.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE,
				.Transition = D3D12_RESOURCE_TRANSITION_BARRIER({
					.pResource = Resource1,
					.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES,
					.StateBefore = Before1, .StateAfter = After1
				})
			})
		};
		GCL->ResourceBarrier(static_cast<UINT>(size(RBs)), data(RBs));
	}
	static void CopyToUploadResource(ID3D12Resource* Resource, const size_t Size, const void* Source, const D3D12_RANGE* Range = nullptr) {
		if (nullptr != Resource && Size && nullptr != Source) [[likely]] {
			BYTE* Data;
			VERIFY_SUCCEEDED(Resource->Map(0, Range, reinterpret_cast<void**>(&Data))); {
				memcpy(Data, Source, Size);
			} Resource->Unmap(0, nullptr);
		}
	}

	void CreateTexture_Depth(const UINT64 Width, const UINT Height) {
		DepthTextures.emplace_back().Create(COM_PTR_GET(Device), Width, Height, 1, D3D12_CLEAR_VALUE({ .Format = DXGI_FORMAT_D24_UNORM_S8_UINT, .DepthStencil = D3D12_DEPTH_STENCIL_VALUE({.Depth = 1.0f, .Stencil = 0 }) }));
	}
	void CreateTexture_Depth() {
		CreateTexture_Depth(static_cast<UINT64>(Rect.right - Rect.left), static_cast<UINT>(Rect.bottom - Rect.top));
	}

	void CreatePipelineState_VsPs_Input(COM_PTR<ID3D12PipelineState>& PST, ID3D12RootSignature* RS, const D3D12_PRIMITIVE_TOPOLOGY_TYPE PTT, const D3D12_RASTERIZER_DESC& RD, const BOOL DepthEnable, const std::vector<D3D12_INPUT_ELEMENT_DESC>& IEDs, const std::array<D3D12_SHADER_BYTECODE, 2>& SBCs) {
		const std::vector RTBDs = {
			D3D12_RENDER_TARGET_BLEND_DESC({
				.BlendEnable = FALSE, .LogicOpEnable = FALSE, 
				.SrcBlend = D3D12_BLEND_ONE, .DestBlend = D3D12_BLEND_ZERO, .BlendOp = D3D12_BLEND_OP_ADD,
				.SrcBlendAlpha = D3D12_BLEND_ONE, .DestBlendAlpha = D3D12_BLEND_ZERO, .BlendOpAlpha = D3D12_BLEND_OP_ADD,
				.LogicOp = D3D12_LOGIC_OP_NOOP, 
				.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL, 
			}),
		};
		constexpr D3D12_DEPTH_STENCILOP_DESC DSOD = {
			.StencilFailOp = D3D12_STENCIL_OP_KEEP,			
			.StencilDepthFailOp = D3D12_STENCIL_OP_KEEP,	
			.StencilPassOp = D3D12_STENCIL_OP_KEEP,			
			.StencilFunc = D3D12_COMPARISON_FUNC_ALWAYS	
		};
		const D3D12_DEPTH_STENCIL_DESC DSD = {
			.DepthEnable = DepthEnable, .DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL, .DepthFunc = D3D12_COMPARISON_FUNC_LESS,
			.StencilEnable = FALSE, .StencilReadMask = D3D12_DEFAULT_STENCIL_READ_MASK, .StencilWriteMask = D3D12_DEFAULT_STENCIL_WRITE_MASK,
			.FrontFace = DSOD, .BackFace = DSOD
		};
		const std::vector RTVs = { DXGI_FORMAT_R8G8B8A8_UNORM };

		Threads.emplace_back(std::thread::thread(DX::CreatePipelineStateVsPsDsHsGs, std::ref(PST), COM_PTR_GET(Device), RS, PTT, RTBDs, RD, DSD, SBCs[0], SBCs[1], NullSBC, NullSBC, NullSBC, IEDs, RTVs, nullptr));
	}
	void CreatePipelineState_VsPs(COM_PTR<ID3D12PipelineState>& PST, ID3D12RootSignature* RS, const D3D12_PRIMITIVE_TOPOLOGY_TYPE PTT, const D3D12_RASTERIZER_DESC& RD, const BOOL DepthEnable, const std::array<D3D12_SHADER_BYTECODE, 2>& SBCs) {
		CreatePipelineState_VsPs_Input(PST, RS, PTT, RD, DepthEnable, {}, SBCs);
	}
	void CreatePipelineState_VsPsGs_Input(COM_PTR<ID3D12PipelineState>& PST, ID3D12RootSignature* RS, const D3D12_PRIMITIVE_TOPOLOGY_TYPE PTT, const D3D12_RASTERIZER_DESC& RD, const BOOL DepthEnable, const std::vector<D3D12_INPUT_ELEMENT_DESC>& IEDs, const std::array<D3D12_SHADER_BYTECODE, 3>& SBCs)
	{
		const std::vector RTBDs = {
			D3D12_RENDER_TARGET_BLEND_DESC({
				.BlendEnable = FALSE, .LogicOpEnable = FALSE,
				.SrcBlend = D3D12_BLEND_ONE, .DestBlend = D3D12_BLEND_ZERO, .BlendOp = D3D12_BLEND_OP_ADD,
				.SrcBlendAlpha = D3D12_BLEND_ONE, .DestBlendAlpha = D3D12_BLEND_ZERO, .BlendOpAlpha = D3D12_BLEND_OP_ADD,
				.LogicOp = D3D12_LOGIC_OP_NOOP,
				.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL,
			}),
		};
		constexpr D3D12_DEPTH_STENCILOP_DESC DSOD = {
			.StencilFailOp = D3D12_STENCIL_OP_KEEP,
			.StencilDepthFailOp = D3D12_STENCIL_OP_KEEP,
			.StencilPassOp = D3D12_STENCIL_OP_KEEP,
			.StencilFunc = D3D12_COMPARISON_FUNC_ALWAYS
		};
		const D3D12_DEPTH_STENCIL_DESC DSD = {
			.DepthEnable = DepthEnable, .DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL, .DepthFunc = D3D12_COMPARISON_FUNC_LESS,
			.StencilEnable = FALSE, .StencilReadMask = D3D12_DEFAULT_STENCIL_READ_MASK, .StencilWriteMask = D3D12_DEFAULT_STENCIL_WRITE_MASK,
			.FrontFace = DSOD, .BackFace = DSOD
		};
		const std::vector RTVs = { DXGI_FORMAT_R8G8B8A8_UNORM };

		Threads.emplace_back(std::thread::thread(DX::CreatePipelineStateVsPsDsHsGs, std::ref(PST), COM_PTR_GET(Device), RS, PTT, RTBDs, RD, DSD, SBCs[0], SBCs[1], NullSBC, NullSBC, SBCs[2], IEDs, RTVs, nullptr));
	}
	void CreatePipelineState_VsPsGs(COM_PTR<ID3D12PipelineState>& PST, ID3D12RootSignature* RS, const D3D12_PRIMITIVE_TOPOLOGY_TYPE PTT, const D3D12_RASTERIZER_DESC& RD, const BOOL DepthEnable, const std::array<D3D12_SHADER_BYTECODE, 3>& SBCs) {
		CreatePipelineState_VsPsGs_Input(PST, RS, PTT, RD, DepthEnable, {}, SBCs);
	}

	void PopulateCommandList_Clear(const size_t i, const DirectX::XMVECTORF32& Color) {
		const auto CL = COM_PTR_GET(DirectCommandLists[i]);
		const auto CA = COM_PTR_GET(DirectCommandAllocators[0]);
		VERIFY_SUCCEEDED(CL->Reset(CA, nullptr)); {
			CL->RSSetViewports(static_cast<UINT>(size(Viewports)), data(Viewports));
			CL->RSSetScissorRects(static_cast<UINT>(size(ScissorRects)), data(ScissorRects));
			const auto SCR = COM_PTR_GET(SwapChainBackBuffers[i].Resource);
			ResourceBarrier(CL, SCR, D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET);
			{
				constexpr std::array<D3D12_RECT, 0> Rects = {};
				CL->ClearRenderTargetView(SwapChainBackBuffers[i].Handle, Color, static_cast<UINT>(size(Rects)), data(Rects));
			}
			ResourceBarrier(CL, SCR, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT);
		} VERIFY_SUCCEEDED(CL->Close());
	}

	class ResourceBase
	{
	public:
		COM_PTR<ID3D12Resource> Resource;
		ResourceBase& Create(ID3D12Device* Device, const size_t Size, const D3D12_HEAP_TYPE HT, const void* Source = nullptr) {
			DX::CreateBufferResource(COM_PTR_PUT(Resource), Device, Size, D3D12_RESOURCE_FLAG_NONE, HT, D3D12_RESOURCE_STATE_GENERIC_READ, Source);
			return *this;
		}
		void PopulateCopyCommand(ID3D12GraphicsCommandList* GCL, const size_t Size, ID3D12Resource* Upload, const D3D12_RESOURCE_STATES RS = D3D12_RESOURCE_STATE_GENERIC_READ) {
			GCL->CopyBufferRegion(COM_PTR_GET(Resource), 0, Upload, 0, Size);
			{
				const std::array RBs = {
					D3D12_RESOURCE_BARRIER({
						.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION,
						.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE,
						.Transition = D3D12_RESOURCE_TRANSITION_BARRIER({
							.pResource = COM_PTR_GET(Resource),
							.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES,
							.StateBefore = D3D12_RESOURCE_STATE_COPY_DEST, .StateAfter = RS
						})
					})
				};
				GCL->ResourceBarrier(static_cast<UINT>(size(RBs)), data(RBs));
			}
		}
		void ExecuteCopyCommand(ID3D12Device* Device, ID3D12CommandAllocator* CA, ID3D12GraphicsCommandList* GCL, ID3D12CommandQueue* CQ, ID3D12Fence* Fence, const size_t Size, const void* Source, const D3D12_RESOURCE_STATES RS = D3D12_RESOURCE_STATE_GENERIC_READ) {
			UploadResource Upload;
			Upload.Create(Device, Size, Source);
			VERIFY_SUCCEEDED(GCL->Reset(CA, nullptr)); {
				PopulateCopyCommand(GCL, Size, COM_PTR_GET(Upload.Resource), RS);
			} VERIFY_SUCCEEDED(GCL->Close());
			DX::ExecuteAndWait(CQ, GCL, Fence);
		}
	};
	class DefaultResource : public ResourceBase
	{
	private:
		using Super = ResourceBase;
	public:
		DefaultResource& Create(ID3D12Device* Device, const size_t Size) {
			DX::CreateBufferResource(COM_PTR_PUT(Resource), Device, Size, D3D12_RESOURCE_FLAG_NONE, D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_STATE_COMMON);
			return *this;
		}
	};
	class UploadResource : public ResourceBase
	{
	private:
		using Super = ResourceBase;
	public:
		UploadResource& Create(ID3D12Device* Device, const size_t Size, const void* Source = nullptr) {
			DX::CreateBufferResource(COM_PTR_PUT(Resource), Device, Size, D3D12_RESOURCE_FLAG_NONE, D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_STATE_GENERIC_READ, Source);
			return *this;
		}
	};
	class VertexBuffer : public DefaultResource
	{
	private:
		using Super = DefaultResource;
	public:
		D3D12_VERTEX_BUFFER_VIEW View;
		VertexBuffer& Create(ID3D12Device* Device, const size_t Size, const UINT Stride) {
			Super::Create(Device, Size);
			View = D3D12_VERTEX_BUFFER_VIEW({ .BufferLocation = Resource->GetGPUVirtualAddress(), .SizeInBytes = static_cast<UINT>(Size), .StrideInBytes = Stride });
			return *this;
		}
	};
	class IndexBuffer : public DefaultResource
	{
	private:
		using Super = DefaultResource;
	public:
		D3D12_INDEX_BUFFER_VIEW View;
		IndexBuffer& Create(ID3D12Device* Device, const size_t Size, const DXGI_FORMAT Format) {
			Super::Create(Device, Size);
			View = D3D12_INDEX_BUFFER_VIEW({ .BufferLocation = Resource->GetGPUVirtualAddress(), .SizeInBytes = static_cast<UINT>(Size), .Format = Format });
			return *this;
		}
	};
	class IndirectBuffer : public DefaultResource
	{
	private:
		using Super = DefaultResource;
	protected:
		IndirectBuffer& Create(ID3D12Device* Device, const size_t Size, const D3D12_INDIRECT_ARGUMENT_TYPE Type) {
			Super::Create(Device, Size);
			const std::array IADs = { D3D12_INDIRECT_ARGUMENT_DESC({.Type = Type }), };
			const D3D12_COMMAND_SIGNATURE_DESC CSD = { .ByteStride = static_cast<UINT>(Size), .NumArgumentDescs = static_cast<const UINT>(size(IADs)), .pArgumentDescs = data(IADs), .NodeMask = 0 };
			Device->CreateCommandSignature(&CSD, nullptr, COM_PTR_UUIDOF_PUTVOID(CommandSignature));
			return *this;
		}
	public:
		COM_PTR<ID3D12CommandSignature> CommandSignature;
		IndirectBuffer& Create(ID3D12Device* Device, const D3D12_DRAW_INDEXED_ARGUMENTS& DIA) { return Create(Device, sizeof(DIA), D3D12_INDIRECT_ARGUMENT_TYPE_DRAW_INDEXED); }
		IndirectBuffer& Create(ID3D12Device* Device, const D3D12_DRAW_ARGUMENTS& DA) { return Create(Device, sizeof(DA), D3D12_INDIRECT_ARGUMENT_TYPE_DRAW); }
		IndirectBuffer& Create(ID3D12Device* Device, const D3D12_DISPATCH_ARGUMENTS& DA) { return Create(Device, sizeof(DA), D3D12_INDIRECT_ARGUMENT_TYPE_DISPATCH); }
	};
	class ConstantBuffer : public UploadResource
	{
	private:
		using Super = UploadResource;
	public:
		ConstantBuffer& Create(ID3D12Device* Device, const size_t Size, const void* Source = nullptr) {
			Super::Create(Device, RoundUp256(Size), Source);
			return *this;
		}
	};
	class TextureBase : public ResourceBase
	{
	private:
		using Super = ResourceBase;
	public:
		void Create(ID3D12Device* Device, const UINT64 Width, const UINT Height, const UINT16 DepthOrArraySize, const DXGI_FORMAT Format) {
			DX::CreateTextureResource(COM_PTR_PUT(Resource), Device, Width, Height, DepthOrArraySize, 1, Format, D3D12_RESOURCE_FLAG_NONE, D3D12_RESOURCE_STATE_COPY_DEST);
		}
	};
	class Texture : public TextureBase
	{
	private:
		using Super = TextureBase;
	public:
		D3D12_SHADER_RESOURCE_VIEW_DESC SRV;
		void Create(ID3D12Device* Device, const UINT64 Width, const UINT Height, const UINT16 DepthOrArraySize, const DXGI_FORMAT Format) {
			CreateTextureResource(COM_PTR_PUT(Resource), Device, Width, Height, DepthOrArraySize, 1, Format, D3D12_RESOURCE_FLAG_NONE, D3D12_RESOURCE_STATE_COPY_DEST);
			SRV = DepthOrArraySize == 1 ?
				D3D12_SHADER_RESOURCE_VIEW_DESC({ .Format = Format, .ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D, .Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING, .Texture2D = D3D12_TEX2D_SRV({.MostDetailedMip = 0, .MipLevels = Resource->GetDesc().MipLevels, .PlaneSlice = 0, .ResourceMinLODClamp = 0.0f }) }) :
				D3D12_SHADER_RESOURCE_VIEW_DESC({ .Format = Format, .ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2DARRAY, .Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING, .Texture2DArray = D3D12_TEX2D_ARRAY_SRV({.MostDetailedMip = 0, .MipLevels = Resource->GetDesc().MipLevels, .FirstArraySlice = 0, .ArraySize = DepthOrArraySize, .PlaneSlice = 0, .ResourceMinLODClamp = 0.0f }) });
		}
	};
	class DepthTexture : public TextureBase
	{
	private:
		using Super = TextureBase;
	public:
		D3D12_DEPTH_STENCIL_VIEW_DESC DSV;
		void Create(ID3D12Device* Device, const UINT64 Width, const UINT Height, const UINT16 DepthOrArraySize, const D3D12_CLEAR_VALUE& CV) {
			DX::CreateRenderTextureResource(COM_PTR_PUT(Resource), Device, Width, Height, DepthOrArraySize, 1, CV, D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL, D3D12_RESOURCE_STATE_DEPTH_WRITE);
			DSV = DepthOrArraySize == 1 ?
				D3D12_DEPTH_STENCIL_VIEW_DESC({ .Format = CV.Format, .ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D, .Flags = D3D12_DSV_FLAG_NONE, .Texture2D = D3D12_TEX2D_DSV({.MipSlice = 0 }) }) :
				D3D12_DEPTH_STENCIL_VIEW_DESC({ .Format = CV.Format, .ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2DARRAY, .Flags = D3D12_DSV_FLAG_NONE, .Texture2DArray = D3D12_TEX2D_ARRAY_DSV({.MipSlice = 0, .FirstArraySlice = 0, .ArraySize = DepthOrArraySize }) });
		}
	};

protected:
	RECT Rect;
	std::vector<std::thread> Threads;

	COM_PTR<IDXGIFactory4> Factory;
	COM_PTR<IDXGIAdapter> Adapter;
	COM_PTR<IDXGIOutput> Output;
	COM_PTR<ID3D12Device> Device;

	COM_PTR<ID3D12CommandQueue> GraphicsCommandQueue;
	
	COM_PTR<ID3D12Fence> GraphicsFence;

	COM_PTR<IDXGISwapChain4> SwapChain;
	COM_PTR<ID3D12DescriptorHeap> SwapChainDescriptorHeap;				
	struct SwapChainBackBuffer
	{
		COM_PTR<ID3D12Resource> Resource;
		D3D12_CPU_DESCRIPTOR_HANDLE Handle;
	};
	std::vector<SwapChainBackBuffer> SwapChainBackBuffers;

	std::vector<COM_PTR<ID3D12CommandAllocator>> DirectCommandAllocators;
	std::vector<COM_PTR<ID3D12GraphicsCommandList>> DirectCommandLists;
	std::vector<COM_PTR<ID3D12CommandAllocator>> BundleCommandAllocators;
	std::vector<COM_PTR<ID3D12GraphicsCommandList>> BundleCommandLists;

	std::vector<VertexBuffer> VertexBuffers;
	std::vector<IndexBuffer> IndexBuffers;
	std::vector<IndirectBuffer> IndirectBuffers;
	std::vector<ConstantBuffer> ConstantBuffers;
	
	std::vector<DepthTexture> DepthTextures;

	std::vector<COM_PTR<ID3D12RootSignature>> RootSignatures;
	
	std::vector<COM_PTR<ID3D12PipelineState>> PipelineStates;

	std::vector<std::pair<COM_PTR<ID3D12DescriptorHeap>, std::vector<D3D12_GPU_DESCRIPTOR_HANDLE>>> CbvSrvUavDescs;
	std::vector<std::pair<COM_PTR<ID3D12DescriptorHeap>, std::vector<D3D12_CPU_DESCRIPTOR_HANDLE>>> RtvDescs;
	std::vector<std::pair<COM_PTR<ID3D12DescriptorHeap>, std::vector<D3D12_CPU_DESCRIPTOR_HANDLE>>> DsvDescs;
	COM_PTR<ID3D12DescriptorHeap> DescHeap;

	std::vector<D3D12_VIEWPORT> Viewports;
	std::vector<D3D12_RECT> ScissorRects;

protected:
	const D3D12_SHADER_BYTECODE NullSBC = { .pShaderBytecode = nullptr, .BytecodeLength = 0 };
};
