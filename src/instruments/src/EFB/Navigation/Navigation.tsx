/* eslint-disable max-len */
import React, { useEffect, useRef, useState } from 'react';
import {
    ArrowClockwise,
    ArrowCounterclockwise,
    ArrowsExpand,
    ArrowsFullscreen,
    Bullseye,
    Dash,
    FullscreenExit,
    MoonFill,
    Plus,
    SunFill,
} from 'react-bootstrap-icons';
import { useSimVar } from '@instruments/common/simVars';
import { useNavigraph } from '../ChartsApi/Navigraph';
import { SimpleInput } from '../UtilComponents/Form/SimpleInput/SimpleInput';
import { useAppDispatch, useAppSelector } from '../Store/store';
import {
    setBoundingBox,
    setChartDimensions,
    setChartLinks,
    setChartName,
    setChartRotation,
    setCurrentPage,
    setSearchQuery,
    setIsFullScreen,
    setPagesViewable,
    setPlaneInFocus,
    setTabIndex,
    setUsingDarkTheme,
} from '../Store/features/navigationPage';
import { PageLink, PageRedirect, TabRoutes } from '../Utils/routing';
import { Navbar } from '../UtilComponents/Navbar';
import { NavigraphNav } from './Pages/NavigraphPage';
import { getPdfUrl, LocalFileChartUI } from './Pages/LocalFilesPage';
import { PinnedChartUI } from './Pages/PinnedCharts';

export const ChartComponent = () => {
    const dispatch = useAppDispatch();
    const {
        chartDimensions,
        chartLinks,
        chartRotation,
        isFullScreen,
        usingDarkTheme,
        planeInFocus,
        boundingBox,
        pagesViewable,
        chartId,
        currentPage,
        provider,
    } = useAppSelector((state) => state.navigationTab);

    const { userName } = useNavigraph();
    const position = useRef({ top: 0, y: 0, left: 0, x: 0 });
    const ref = useRef<HTMLDivElement>(null);

    const chartRef = useRef<HTMLDivElement>(null);

    const [aircraftIconVisible, setAircraftIconVisible] = useState(false);
    const [aircraftIconPosition, setAircraftIconPosition] = useState<{ x: number, y: number, r: number }>({ x: 0, y: 0, r: 0 });
    const [aircraftLatitude] = useSimVar('PLANE LATITUDE', 'degree latitude', 1000);
    const [aircraftLongitude] = useSimVar('PLANE LONGITUDE', 'degree longitude', 1000);
    const [aircraftTrueHeading] = useSimVar('PLANE HEADING DEGREES TRUE', 'degrees', 100);

    useEffect(() => {
        let visible = false;

        console.log(boundingBox, aircraftLatitude, aircraftLongitude);

        if (boundingBox
            && aircraftLatitude >= boundingBox.bottomLeft.lat
            && aircraftLatitude <= boundingBox.topRight.lat
            && aircraftLongitude >= boundingBox.bottomLeft.lon
            && aircraftLongitude <= boundingBox.topRight.lon) {
            const dx = boundingBox.topRight.xPx - boundingBox.bottomLeft.xPx;
            const dy = boundingBox.bottomLeft.yPx - boundingBox.topRight.yPx;
            const dLat = boundingBox.topRight.lat - boundingBox.bottomLeft.lat;
            const dLon = boundingBox.topRight.lon - boundingBox.bottomLeft.lon;
            const x = boundingBox.bottomLeft.xPx + dx * ((aircraftLongitude - boundingBox.bottomLeft.lon) / dLon);
            const y = boundingBox.topRight.yPx + dy * ((boundingBox.topRight.lat - aircraftLatitude) / dLat);

            setAircraftIconPosition({ x, y, r: aircraftTrueHeading });
            visible = true;
        }

        setAircraftIconVisible(visible);
    }, [boundingBox, chartLinks, aircraftLatitude.toFixed(2), aircraftLongitude.toFixed(2), aircraftTrueHeading]);

    useEffect(() => {
        const { width, height } = chartDimensions;

        if (chartRef.current) {
            if (width) {
                chartRef.current.style.width = `${width}px`;
            }

            if (height) {
                chartRef.current.style.height = `${height}px`;
            }
        }
    }, [chartRef, chartDimensions]);

    useEffect(() => {
        if (planeInFocus) {
            dispatch(setChartRotation(360 - aircraftIconPosition.r));
            // TODO: implement the chart translation
            // if (ref.current) {
            //     ref.current.scrollTop = aircraftIconPosition.y + ((ref.current.clientHeight - aircraftIconPosition.y) / 2);
            //     ref.current.scrollLeft = -(ref.current.clientWidth - aircraftIconPosition.x) / 2;
            // }
        }
    }, [aircraftIconPosition.r, planeInFocus]);

    const handleMouseDown = (event: React.MouseEvent<HTMLDivElement>) => {
        position.current.top = ref.current ? ref.current.scrollTop : 0;
        position.current.y = event.clientY;
        position.current.left = ref.current ? ref.current.scrollLeft : 0;
        position.current.x = event.clientX;

        document.addEventListener('mousemove', mouseMoveHandler);
        document.addEventListener('mouseup', mouseUpHandler);
    };

    const mouseMoveHandler = (event) => {
        const dy = event.clientY - position.current.y;
        const dx = event.clientX - position.current.x;
        if (ref.current) {
            ref.current.scrollTop = position.current.top - dy;
            ref.current.scrollLeft = position.current.left - dx;
        }
    };

    const mouseUpHandler = () => {
        document.removeEventListener('mousemove', mouseMoveHandler);
        document.removeEventListener('mouseup', mouseUpHandler);
    };

    const handleZoomIn = () => {
        if (!chartRef.current) return;

        const currentHeight = chartRef.current.clientHeight;
        const currentWidth = chartRef.current.clientWidth;
        if (currentHeight >= 2500) return;

        dispatch(setChartDimensions({ height: currentHeight * 1.1, width: currentWidth * 1.1 }));
    };

    const handleZoomOut = () => {
        if (!chartRef.current) return;

        const currentHeight = chartRef.current!.clientHeight;
        const currenWidth = chartRef.current!.clientWidth;
        if (currentHeight <= 775) return;

        dispatch(setChartDimensions({ height: currentHeight * 0.9, width: currenWidth * 0.9 }));
    };

    const expandToHeight = () => {
        if (!ref.current || !chartRef.current) return;

        const scale = ref.current.clientHeight / chartRef.current.clientHeight;

        dispatch(setChartDimensions({ width: (chartDimensions.width ?? 0) * scale, height: ref.current!.clientHeight }));
    };

    const expandToWidth = () => {
        if (!ref.current || !chartRef.current) return;

        const scale = ref.current.clientWidth / chartRef.current.clientWidth;

        dispatch(setChartDimensions({ width: ref.current!.clientWidth, height: (chartDimensions.height ?? 0) * scale }));
    };

    // The functions that handle rotation get the closest 45 degree angle increment to the current angle
    const handleRotateRight = () => {
        dispatch(setChartRotation(chartRotation + (45 - chartRotation % 45)));
    };

    const handleRotateLeft = () => {
        dispatch(setChartRotation(chartRotation - (45 + chartRotation % 45)));
    };

    useEffect(() => {
        if (!chartDimensions.height && !chartDimensions.width) {
            const img = new Image();
            img.onload = function () {
                if (ref.current) {
                    // @ts-ignore
                    // eslint-disable-next-line react/no-this-in-sfc
                    dispatch(setChartDimensions({ width: this.width * (ref.current.clientHeight / this.height), height: ref.current.clientHeight }));
                }
            };
            img.src = chartLinks.light;
        }
    }, [chartLinks]);

    useEffect(() => {
        setCurrentPage(1);
    }, [chartId]);

    useEffect(() => {
        if (pagesViewable > 1) {
            getPdfUrl(chartId, currentPage).then((url) => {
                dispatch(setChartName({ light: url, dark: url }));
            });
        }
    }, [currentPage]);

    if (!chartLinks.light || !chartLinks.dark) {
        return (
            <div
                className={`flex relative items-center justify-center bg-theme-accent rounded-lg ${!isFullScreen && 'rounded-l-none ml-6'}`}
                style={{ width: `${isFullScreen ? '1278px' : '804px'}` }}
            >
                {isFullScreen && (
                    <div
                        className="flex absolute top-6 right-6 flex-row items-center p-4 hover:text-theme-body bg-theme-secondary hover:bg-theme-highlight rounded-md transition duration-100"
                        onClick={() => dispatch(setIsFullScreen(false))}
                    >
                        <FullscreenExit size={40} />
                        <p className="ml-4 text-current">Exit Fullscreen Mode</p>
                    </div>
                )}
                <p>There is no chart to display.</p>
            </div>
        );
    }

    return (
        <div
            className={`relative ${!isFullScreen && 'rounded-l-none ml-6'}`}
            style={{ width: `${isFullScreen ? '1278px' : '804px'}` }}
        >
            {pagesViewable > 1 && (
                <div className="flex overflow-hidden absolute top-6 left-6 z-40 flex-row items-center rounded-md">
                    <div
                        className={`flex flex-row justify-center items-center h-14 bg-opacity-40 transition duration-100 cursor-pointer hover:text-theme-body bg-theme-secondary hover:bg-theme-highlight ${currentPage === 1 && 'opacity-50 pointer-events-none'}`}
                        onClick={() => dispatch(setCurrentPage(currentPage - 1))}
                    >
                        <Dash size={40} />
                    </div>
                    <SimpleInput
                        min={1}
                        max={pagesViewable}
                        value={currentPage}
                        number
                        onBlur={(value) => {
                            dispatch(setCurrentPage(Number.parseInt(value)));
                        }}
                        className="w-16 h-14 rounded-r-none rounded-l-none border-transparent"
                    />
                    <div className="flex flex-shrink-0 items-center px-2 h-14 bg-theme-secondary">
                        of
                        {' '}
                        {pagesViewable}
                    </div>
                    <div
                        className={`flex flex-row justify-center items-center h-14 bg-opacity-40 transition duration-100 cursor-pointer hover:text-theme-body bg-theme-secondary hover:bg-theme-highlight ${currentPage === pagesViewable && 'opacity-50 pointer-events-none'}`}
                        onClick={() => dispatch(setCurrentPage(currentPage + 1))}
                    >

                        <Plus size={40} />
                    </div>
                </div>
            )}

            <div className="flex overflow-hidden absolute top-6 right-6 bottom-6 z-30 flex-col justify-between rounded-md cursor-pointer">
                <div className="flex overflow-hidden flex-col rounded-md">
                    <button
                        type="button"
                        onClick={handleRotateLeft}
                        className={`p-2 transition hover:text-theme-body duration-100 cursor-pointer bg-theme-secondary hover:bg-theme-highlight ${planeInFocus && 'text-theme-unselected pointer-events-none'}`}
                    >
                        <ArrowCounterclockwise size={40} />
                    </button>
                    {boundingBox && (
                        <button
                            type="button"
                            onClick={() => dispatch(setPlaneInFocus(!planeInFocus))}
                            className={`p-2 transition hover:text-theme-body duration-100 cursor-pointer bg-theme-secondary hover:bg-theme-highlight ${planeInFocus && 'text-theme-highlight  hover:text-theme-text'}`}
                        >
                            <Bullseye size={40} />
                        </button>
                    )}
                    <button
                        type="button"
                        onClick={handleRotateRight}
                        className={`p-2 transition hover:text-theme-body duration-100 cursor-pointer bg-theme-secondary hover:bg-theme-highlight ${planeInFocus && 'text-theme-unselected pointer-events-none'}`}
                    >
                        <ArrowClockwise className="fill-current" size={40} />
                    </button>
                </div>
                <div className="flex overflow-hidden flex-col rounded-md">
                    <button
                        type="button"
                        onClick={expandToHeight}
                        className="p-2 hover:text-theme-body bg-theme-secondary hover:bg-theme-highlight transition duration-100 cursor-pointer"
                    >
                        <ArrowsExpand size={40} />
                    </button>
                    <button
                        type="button"
                        onClick={expandToWidth}
                        className="p-2 hover:text-theme-body bg-theme-secondary hover:bg-theme-highlight transition duration-100 cursor-pointer"
                    >
                        <ArrowsExpand className="transform rotate-90" size={40} />
                    </button>

                    <button
                        type="button"
                        onClick={handleZoomIn}
                        className="p-2 hover:text-theme-body bg-theme-secondary hover:bg-theme-highlight transition duration-100 cursor-pointer"
                    >
                        <Plus size={40} />
                    </button>
                    <button
                        type="button"
                        onClick={handleZoomOut}
                        className="p-2 hover:text-theme-body bg-theme-secondary hover:bg-theme-highlight transition duration-100 cursor-pointer"
                    >
                        <Dash size={40} />
                    </button>
                </div>
                <div className="flex overflow-hidden flex-col rounded-md">
                    <div
                        className="p-2 hover:text-theme-body bg-theme-secondary hover:bg-theme-highlight rounded-md transition duration-100 cursor-pointer"
                        onClick={() => {
                            if (chartRef.current && ref.current) {
                                if (chartRef.current.clientWidth === ref.current.clientWidth) {
                                    const width = isFullScreen ? 804 : 1278;

                                    const scale = width / (chartDimensions.width ?? 0);
                                    const height = (chartDimensions.height ?? 0) * scale;

                                    dispatch(setChartDimensions({ width, height }));
                                }
                            }
                            dispatch(setIsFullScreen(!isFullScreen));
                        }}
                    >
                        {isFullScreen
                            ? <FullscreenExit size={40} />
                            : <ArrowsFullscreen size={40} />}
                    </div>

                    {provider === 'NAVIGRAPH' && (
                        <div
                            className="p-2 mt-3 hover:text-theme-body bg-theme-secondary hover:bg-theme-highlight rounded-md transition duration-100 cursor-pointer"
                            onClick={() => dispatch(setUsingDarkTheme(!usingDarkTheme))}
                        >
                            {!usingDarkTheme ? <MoonFill size={40} /> : <SunFill size={40} />}
                        </div>
                    )}
                </div>
            </div>

            <div
                className="flex overflow-x-hidden overflow-y-scroll relative flex-row mx-auto h-full bg-theme-accent rounded-lg grabbable no-scrollbar"
                ref={ref}
                onMouseDown={handleMouseDown}
            >
                <div
                    className="relative m-auto transition duration-100"
                    style={{ transform: `rotate(${chartRotation}deg)` }}
                >
                    {(chartLinks && provider === 'NAVIGRAPH') && (
                        <p
                            className="absolute top-0 left-0 font-bold text-theme-highlight whitespace-nowrap transition duration-100 transform -translate-y-full"
                        >
                            This chart is linked to
                            {' '}
                            {userName}
                        </p>
                    )}

                    { (aircraftIconVisible && boundingBox) && (
                        <svg viewBox={`0 0 ${boundingBox.width} ${boundingBox.height}`} className="absolute top-0 left-0 z-30">
                            <g
                                className="transition duration-100"
                                transform={`translate(${aircraftIconPosition.x} ${aircraftIconPosition.y}) rotate(${aircraftIconPosition.r})`}
                                strokeLinecap="square"
                            >
                                <path d="M-20,0 L20,0" stroke="black" strokeWidth="7" />
                                <path d="M-10,20 L10,20" stroke="black" strokeWidth="7" />
                                <path d="M0,-10 L0,30" stroke="black" strokeWidth="7" />
                                <path d="M-20,0 L20,0" stroke="yellow" strokeWidth="5" />
                                <path d="M-10,20 L10,20" stroke="yellow" strokeWidth="5" />
                                <path d="M0,-10 L0,30" stroke="yellow" strokeWidth="5" />
                            </g>
                        </svg>
                    )}

                    <div ref={chartRef}>
                        <img
                            className="absolute left-0 w-full transition duration-100 select-none"
                            draggable={false}
                            src={chartLinks.dark}
                            alt="chart"
                        />
                        <img
                            className={`absolute left-0 w-full transition duration-100 select-none ${usingDarkTheme && 'opacity-0'}`}
                            draggable={false}
                            src={chartLinks.light}
                            alt="chart"
                        />
                    </div>
                </div>
            </div>
        </div>
    );
};

const tabs: PageLink[] = [
    { name: 'Local Files', component: <LocalFileChartUI /> },
    { name: 'Navigraph', component: <NavigraphNav /> },
    { name: 'Pinned Charts', component: <PinnedChartUI /> },
];

export const Navigation = () => {
    const dispatch = useAppDispatch();

    return (
        <div className="w-full h-full">
            <div className="relative">
                <h1 className="font-bold">Navigation & Charts</h1>
                <Navbar
                    className="absolute top-0 right-0"
                    tabs={tabs}
                    basePath="/navigation"
                    onSelected={() => {
                        dispatch(setChartLinks({ light: '', dark: '' }));
                        dispatch(setChartName({ light: '', dark: '' }));
                        dispatch(setBoundingBox(undefined));
                        dispatch(setTabIndex(0));
                        dispatch(setPagesViewable(1));
                        dispatch(setCurrentPage(1));
                        dispatch(setSearchQuery(''));
                    }}
                />
            </div>

            <div className="mt-4">
                <PageRedirect basePath="/navigation" tabs={tabs} />
                <TabRoutes basePath="/navigation" tabs={tabs} />
            </div>
        </div>
    );
};
