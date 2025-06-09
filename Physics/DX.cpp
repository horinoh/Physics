#include "DX.h"

#pragma comment(lib, "d3d12.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3dcompiler.lib")

template<> const DirectX::XMFLOAT3 GetMin(const DirectX::XMFLOAT3& lhs, const DirectX::XMFLOAT3& rhs) { return DirectX::XMFLOAT3((std::min)(lhs.x, rhs.x), (std::min)(lhs.y, rhs.y), (std::min)(lhs.z, rhs.z)); }
template<> const DirectX::XMFLOAT3 GetMax(const DirectX::XMFLOAT3& lhs, const DirectX::XMFLOAT3& rhs) { return DirectX::XMFLOAT3((std::max)(lhs.x, rhs.x), (std::max)(lhs.y, rhs.y), (std::max)(lhs.z, rhs.z)); }
template<>
void AdjustScale(std::vector<DirectX::XMFLOAT3>& Vertices, const float Scale)
{
	auto Max = (std::numeric_limits<DirectX::XMFLOAT3>::min)();
	auto Min = (std::numeric_limits<DirectX::XMFLOAT3>::max)();
	for (const auto& i : Vertices) {
		Min = GetMin(Min, i);
		Max = GetMax(Max, i);
	}
	const auto Diff = DirectX::XMFLOAT3(Max.x - Min.x, Max.y - Min.y, Max.z - Min.z);
	const auto Bound = (std::max)((std::max)(Diff.x, Diff.y), Diff.z);
	const auto Coef = Scale / Bound;
	std::ranges::transform(Vertices, std::begin(Vertices), [&](const DirectX::XMFLOAT3& rhs) { return DirectX::XMFLOAT3(rhs.x * Coef, (rhs.y - Diff.y * 0.5f) * Coef, (rhs.z - Min.z) * Coef); });
}

void DX::CreateDevice([[maybe_unused]] HWND hWnd)
{
	VERIFY_SUCCEEDED(CreateDXGIFactory1(COM_PTR_UUIDOF_PUTVOID(Factory)));

	//!< アダプター(GPU)の選択
	std::vector<DXGI_ADAPTER_DESC> ADs;
	for (UINT i = 0; DXGI_ERROR_NOT_FOUND != Factory->EnumAdapters(i, COM_PTR_PUT(Adapter)); ++i) {
		VERIFY_SUCCEEDED(Adapter->GetDesc(&ADs.emplace_back()));
		COM_PTR_RESET(Adapter);
	}
	const auto Index = static_cast<UINT>(std::distance(begin(ADs), std::ranges::max_element(ADs, [](const DXGI_ADAPTER_DESC& lhs, const DXGI_ADAPTER_DESC& rhs) { return lhs.DedicatedSystemMemory > rhs.DedicatedSystemMemory; })));
	VERIFY_SUCCEEDED(Factory->EnumAdapters(Index, COM_PTR_PUT(Adapter)));

	//!< アウトプット(Moniter)の選択
	COM_PTR<IDXGIAdapter> DA;
	for (UINT i = 0; DXGI_ERROR_NOT_FOUND != Factory->EnumAdapters(i, COM_PTR_PUT(DA)); ++i) {
		for (UINT j = 0; DXGI_ERROR_NOT_FOUND != DA->EnumOutputs(j, COM_PTR_PUT(Output)); ++j) {
			if (nullptr != Output) { break; }
			COM_PTR_RESET(Output);
		}
		COM_PTR_RESET(DA);
		if (nullptr != Output) { break; }
	}

	//!< 高フィーチャーレベル優先でデバイスを作成
	constexpr std::array FeatureLevels = {
		D3D_FEATURE_LEVEL_12_2,
		D3D_FEATURE_LEVEL_12_1,
		D3D_FEATURE_LEVEL_12_0,
		D3D_FEATURE_LEVEL_11_1,
		D3D_FEATURE_LEVEL_11_0,
		D3D_FEATURE_LEVEL_10_1,
		D3D_FEATURE_LEVEL_10_0,
		D3D_FEATURE_LEVEL_9_3,
		D3D_FEATURE_LEVEL_9_2,
		D3D_FEATURE_LEVEL_9_1,
	};
	for (const auto i : FeatureLevels) {
		if (SUCCEEDED(D3D12CreateDevice(COM_PTR_GET(Adapter), i, COM_PTR_UUIDOF_PUTVOID(Device)))) {
			//!< NumFeatureLevels, pFeatureLevelsRequested は入力、MaxSupportedFeatureLevel は出力となる (NumFeatureLevels, pFeatureLevelsRequested is input, MaxSupportedFeatureLevel is output)
			D3D12_FEATURE_DATA_FEATURE_LEVELS FDFL = { .NumFeatureLevels = static_cast<UINT>(std::size(FeatureLevels)), .pFeatureLevelsRequested = std::data(FeatureLevels) };
			VERIFY_SUCCEEDED(Device->CheckFeatureSupport(D3D12_FEATURE_FEATURE_LEVELS, reinterpret_cast<void*>(&FDFL), sizeof(FDFL)));
#define D3D_FEATURE_LEVEL_ENTRY(fl) case D3D_FEATURE_LEVEL_##fl: break;
			switch (FDFL.MaxSupportedFeatureLevel) {
			default: VERIFY(false); break;
				D3D_FEATURE_LEVEL_ENTRY(12_2)
				D3D_FEATURE_LEVEL_ENTRY(12_1)
				D3D_FEATURE_LEVEL_ENTRY(12_0)
				D3D_FEATURE_LEVEL_ENTRY(11_1)
				D3D_FEATURE_LEVEL_ENTRY(11_0)
				D3D_FEATURE_LEVEL_ENTRY(10_1)
				D3D_FEATURE_LEVEL_ENTRY(10_0)
				D3D_FEATURE_LEVEL_ENTRY(9_3)
				D3D_FEATURE_LEVEL_ENTRY(9_2)
				D3D_FEATURE_LEVEL_ENTRY(9_1)
			}
#undef D3D_FEATURE_LEVEL_ENTRY
			break;
		}
	}
}
void DX::CreateCommandQueue()
{
	constexpr D3D12_COMMAND_QUEUE_DESC CQD = {
		.Type = D3D12_COMMAND_LIST_TYPE_DIRECT,
		.Priority = D3D12_COMMAND_QUEUE_PRIORITY_NORMAL,
		.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE,
		.NodeMask = 0
	};
	VERIFY_SUCCEEDED(Device->CreateCommandQueue(&CQD, COM_PTR_UUIDOF_PUTVOID(GraphicsCommandQueue)));
}
void DX::CreateSwapChain(HWND hWnd, const DXGI_FORMAT ColorFormat, const UINT Width, const UINT Height)
{
	std::vector<DXGI_SAMPLE_DESC> SDs;
	for (UINT i = 1; i <= D3D12_MAX_MULTISAMPLE_SAMPLE_COUNT; ++i) {
		auto FDMSQL = D3D12_FEATURE_DATA_MULTISAMPLE_QUALITY_LEVELS({
			.Format = ColorFormat,
			.SampleCount = i,
			.Flags = D3D12_MULTISAMPLE_QUALITY_LEVELS_FLAG_NONE,
			.NumQualityLevels = 0
			});
		VERIFY_SUCCEEDED(Device->CheckFeatureSupport(D3D12_FEATURE_MULTISAMPLE_QUALITY_LEVELS, reinterpret_cast<void*>(&FDMSQL), sizeof(FDMSQL)));
		if (FDMSQL.NumQualityLevels) {
			SDs.emplace_back(DXGI_SAMPLE_DESC({ .Count = FDMSQL.SampleCount, .Quality = FDMSQL.NumQualityLevels - 1 }));
		}
	}

	constexpr UINT BufferCount = 3;

	auto SCD = DXGI_SWAP_CHAIN_DESC({
		.BufferDesc = DXGI_MODE_DESC({
			.Width = Width, .Height = Height,
			.RefreshRate = DXGI_RATIONAL({.Numerator = 60, .Denominator = 1 }),
			.Format = ColorFormat,
			.ScanlineOrdering = DXGI_MODE_SCANLINE_ORDER_UNSPECIFIED,
			.Scaling = DXGI_MODE_SCALING_UNSPECIFIED
		}),
		.SampleDesc = SDs[0],
		.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT,
		.BufferCount = BufferCount,
		.OutputWindow = hWnd,
		.Windowed = TRUE,
		.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD,
		.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH
	});
	COM_PTR_RESET(SwapChain);
	COM_PTR<IDXGISwapChain> NewSwapChain;
	VERIFY_SUCCEEDED(Factory->CreateSwapChain(COM_PTR_GET(GraphicsCommandQueue), &SCD, COM_PTR_PUT(NewSwapChain)));
	COM_PTR_AS(NewSwapChain, SwapChain);

	const auto DHD = D3D12_DESCRIPTOR_HEAP_DESC({
		.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV,
		.NumDescriptors = SCD.BufferCount,
		.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE,
		.NodeMask = 0
	});
	VERIFY_SUCCEEDED(Device->CreateDescriptorHeap(&DHD, COM_PTR_UUIDOF_PUTVOID(SwapChainDescriptorHeap)));
}
void DX::GetSwapChainResource()
{
	DXGI_SWAP_CHAIN_DESC1 SCD;
	SwapChain->GetDesc1(&SCD);

	auto CDH = SwapChainDescriptorHeap->GetCPUDescriptorHandleForHeapStart();
	const auto IncSize = Device->GetDescriptorHandleIncrementSize(SwapChainDescriptorHeap->GetDesc().Type);
	for (UINT i = 0; i < SCD.BufferCount; ++i) {
		auto& SCBB = SwapChainBackBuffers.emplace_back();
		VERIFY_SUCCEEDED(SwapChain->GetBuffer(i, COM_PTR_UUIDOF_PUTVOID(SCBB.Resource)));
		Device->CreateRenderTargetView(COM_PTR_GET(SCBB.Resource), nullptr, CDH);
		SCBB.Handle = CDH;

		CDH.ptr += IncSize;
	}
}
void DX::CreateDirectCommandList(const size_t Num)
{
	VERIFY_SUCCEEDED(Device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, COM_PTR_UUIDOF_PUTVOID(DirectCommandAllocators.emplace_back())));

	const auto DCA = DirectCommandAllocators[0];
	DXGI_SWAP_CHAIN_DESC1 SCD;
	SwapChain->GetDesc1(&SCD);
	for (UINT i = 0; i < SCD.BufferCount; ++i) {
		VERIFY_SUCCEEDED(Device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, COM_PTR_GET(DCA), nullptr, COM_PTR_UUIDOF_PUTVOID(DirectCommandLists.emplace_back())));
		VERIFY_SUCCEEDED(DirectCommandLists.back()->Close());
	}
}
void DX::CreateBundleCommandList(const size_t Num)
{
	VERIFY_SUCCEEDED(Device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_BUNDLE, COM_PTR_UUIDOF_PUTVOID(BundleCommandAllocators.emplace_back())));

	const auto BCA = BundleCommandAllocators[0];
	for (UINT i = 0; i < Num; ++i) {
		VERIFY_SUCCEEDED(Device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_BUNDLE, COM_PTR_GET(BCA), nullptr, COM_PTR_UUIDOF_PUTVOID(BundleCommandLists.emplace_back())));
		VERIFY_SUCCEEDED(BundleCommandLists.back()->Close());
	}
}

template<> void DX::SerializeRootSignature(COM_PTR<ID3DBlob>& Blob, const std::vector<D3D12_ROOT_PARAMETER1>& RPs, const std::vector<D3D12_STATIC_SAMPLER_DESC>& SSDs, const D3D12_ROOT_SIGNATURE_FLAGS Flags)
{
	D3D12_FEATURE_DATA_ROOT_SIGNATURE FDRS = { .HighestVersion = D3D_ROOT_SIGNATURE_VERSION_1_1 };
	VERIFY_SUCCEEDED(Device->CheckFeatureSupport(D3D12_FEATURE_ROOT_SIGNATURE, reinterpret_cast<void*>(&FDRS), sizeof(FDRS)));
	LOG(std::data(std::format("RootSignature HighestVersion = {:#x}\n", static_cast<UINT>(FDRS.HighestVersion))));
	VERIFY(D3D_ROOT_SIGNATURE_VERSION_1_1 == FDRS.HighestVersion);

	COM_PTR<ID3DBlob> ErrorBlob;
	const D3D12_ROOT_SIGNATURE_DESC1 RSD = {
		.NumParameters = static_cast<UINT>(std::size(RPs)), .pParameters = std::data(RPs),
		.NumStaticSamplers = static_cast<UINT>(std::size(SSDs)), .pStaticSamplers = std::data(SSDs),
		.Flags = Flags
	};
	const D3D12_VERSIONED_ROOT_SIGNATURE_DESC VRSD = { .Version = D3D_ROOT_SIGNATURE_VERSION_1_1, .Desc_1_1 = RSD, };
	VERIFY_SUCCEEDED(D3D12SerializeVersionedRootSignature(&VRSD, COM_PTR_PUT(Blob), COM_PTR_PUT(ErrorBlob)));
}

void DX::CreatePipelineStateVsPsDsHsGs(COM_PTR<ID3D12PipelineState>& PST,
	ID3D12Device* Device, ID3D12RootSignature* RS,
	const D3D12_PRIMITIVE_TOPOLOGY_TYPE PTT,
	const std::vector<D3D12_RENDER_TARGET_BLEND_DESC>& RTBDs,
	const D3D12_RASTERIZER_DESC& RD,
	const D3D12_DEPTH_STENCIL_DESC& DSD,
	const D3D12_SHADER_BYTECODE VS, const D3D12_SHADER_BYTECODE PS, const D3D12_SHADER_BYTECODE DS, const D3D12_SHADER_BYTECODE HS, const D3D12_SHADER_BYTECODE GS,
	const std::vector<D3D12_INPUT_ELEMENT_DESC>& IEDs,
	const std::vector<DXGI_FORMAT>& RTVFormats,
	LPCWSTR Name)
{
	VERIFY((VS.pShaderBytecode != nullptr && VS.BytecodeLength));

	//!< キャッシュドパイプラインステート (CachedPipelineState)
	//!< (VK の VkGraphicsPipelineCreateInfo.basePipelineHandle, basePipelineIndex 相当?)
	COM_PTR<ID3DBlob> CachedBlob;
	if (nullptr != PST) {
		VERIFY_SUCCEEDED(PST->GetCachedBlob(COM_PTR_PUT(CachedBlob)));
	}

	//!< DXでは「パッチコントロールポイント」個数の指定はIASetPrimitiveTopology()の引数として「コマンドリスト作成時」に指定する、VKとは結構異なるので注意
	//!< GCL->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_1_CONTROL_POINT_PATCHLIST);	

	D3D12_GRAPHICS_PIPELINE_STATE_DESC GPSD = {
		.pRootSignature = RS,
		.VS = VS, .PS = PS, .DS = DS, .HS = HS, .GS = GS,
		.StreamOutput = D3D12_STREAM_OUTPUT_DESC({
			.pSODeclaration = nullptr, .NumEntries = 0,
			.pBufferStrides = nullptr, .NumStrides = 0,
			.RasterizedStream = 0
		}),
		.BlendState = D3D12_BLEND_DESC({
			.AlphaToCoverageEnable = TRUE,		//!< マルチサンプルを考慮したアルファテスト(AlphaToCoverageEnable)、アルファが0の箇所には無駄に書き込まない
			.IndependentBlendEnable = FALSE,	//!< マルチレンダーターゲットにそれぞれ別のブレンドステートを割り当てる(IndependentBlendEnable)
			.RenderTarget = {}
		}),
		.SampleMask = D3D12_DEFAULT_SAMPLE_MASK,
		.RasterizerState = RD,
		.DepthStencilState = DSD,
		.InputLayout = D3D12_INPUT_LAYOUT_DESC({.pInputElementDescs = std::data(IEDs), .NumElements = static_cast<UINT>(std::size(IEDs)) }),
		.IBStripCutValue = D3D12_INDEX_BUFFER_STRIP_CUT_VALUE_DISABLED,
		.PrimitiveTopologyType = PTT,
		.NumRenderTargets = static_cast<UINT>(std::size(RTVFormats)), .RTVFormats = {},
		.DSVFormat = DSD.DepthEnable ? DXGI_FORMAT_D24_UNORM_S8_UINT : DXGI_FORMAT_UNKNOWN,
		.SampleDesc = DXGI_SAMPLE_DESC({.Count = 1, .Quality = 0 }),
		.NodeMask = 0, //!< マルチGPUの場合に使用(1つしか使わない場合は0で良い)
		.CachedPSO = D3D12_CACHED_PIPELINE_STATE({.pCachedBlob = nullptr != CachedBlob ? CachedBlob->GetBufferPointer() : nullptr, .CachedBlobSizeInBytes = nullptr != CachedBlob ? CachedBlob->GetBufferSize() : 0 }),
		.Flags = D3D12_PIPELINE_STATE_FLAG_NONE
	};

	//!< レンダーターゲット数分だけ必要なもの
	VERIFY(std::size(RTBDs) <= _countof(GPSD.BlendState.RenderTarget));
	std::ranges::copy(RTBDs, GPSD.BlendState.RenderTarget);
	//!< TRUE == IndependentBlendEnable の場合はレンダーターゲットの分だけ用意すること (If TRUE == IndependentBlendEnable, need NumRenderTarget elements)
	VERIFY((false == GPSD.BlendState.IndependentBlendEnable || std::size(RTBDs) == GPSD.NumRenderTargets) && "");
	VERIFY(GPSD.NumRenderTargets <= _countof(GPSD.RTVFormats));
	std::ranges::copy(RTVFormats, GPSD.RTVFormats);

	VERIFY((0 == GPSD.DS.BytecodeLength || 0 == GPSD.HS.BytecodeLength || GPSD.PrimitiveTopologyType == D3D12_PRIMITIVE_TOPOLOGY_TYPE_PATCH) && "");

	VERIFY_SUCCEEDED(Device->CreateGraphicsPipelineState(&GPSD, COM_PTR_UUIDOF_PUTVOID(PST)));
}

void DX::CreateViewport(const FLOAT Width, const FLOAT Height, const FLOAT MinDepth, const FLOAT MaxDepth)
{
	//!< DirectX、OpenGLは「左下」が原点 (Vulkan はデフォルトで「左上」が原点)
	Viewports = {
		D3D12_VIEWPORT({.TopLeftX = 0.0f, .TopLeftY = 0.0f, .Width = Width, .Height = Height, .MinDepth = MinDepth, .MaxDepth = MaxDepth }),
	};
	//!< left, top, right, bottomで指定 (offset, extentで指定のVKとは異なるので注意)
	ScissorRects = {
		D3D12_RECT({.left = 0, .top = 0, .right = static_cast<LONG>(Width), .bottom = static_cast<LONG>(Height) }),
	};
}

void DX::WaitForFence(ID3D12CommandQueue* CQ, ID3D12Fence* Fence)
{
	auto Value = Fence->GetCompletedValue();
	++Value;
	//!< GPUが到達すれば Value になる
	VERIFY_SUCCEEDED(CQ->Signal(Fence, Value));
	if (Fence->GetCompletedValue() < Value) {
		auto hEvent = CreateEventEx(nullptr, nullptr, 0, EVENT_ALL_ACCESS);
		if (nullptr != hEvent) [[likely]] {
			//!< GetCompletedValue() が FenceValue になったらイベントが発行されるようにする
			VERIFY_SUCCEEDED(Fence->SetEventOnCompletion(Value, hEvent));
			//!< イベント発行まで待つ
			WaitForSingleObject(hEvent, INFINITE);
			CloseHandle(hEvent);
		}
	}
}
void DX::SubmitGraphics(const UINT i)
{
	const std::array CLs = { static_cast<ID3D12CommandList*>(COM_PTR_GET(DirectCommandLists[i])) };
	GraphicsCommandQueue->ExecuteCommandLists(static_cast<UINT>(std::size(CLs)), std::data(CLs));
}
void DX::Present()
{
	VERIFY_SUCCEEDED(SwapChain->Present(1, 0));
}
void DX::Draw()
{
	WaitForFence(COM_PTR_GET(GraphicsCommandQueue), COM_PTR_GET(GraphicsFence));

	const auto Index = SwapChain->GetCurrentBackBufferIndex();
	DrawFrame(Index);
	SubmitGraphics(Index);

	Present();
}
