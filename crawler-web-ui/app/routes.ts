import { type RouteConfig, index, route } from "@react-router/dev/routes";

export default [
    index("routes/home.tsx"),
    route("manual-control", "routes/manualControl.tsx"),
] satisfies RouteConfig;
