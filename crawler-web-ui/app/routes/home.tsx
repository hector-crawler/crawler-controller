import type { Route } from "./+types/home";
import { Link } from "react-router";

export function meta({}: Route.MetaArgs) {
  return [
    { title: "Home | Crawler" },
  ];
}

export default function Home() {
  return (
    <Link to={"/manual-control"}>Manual Control</Link>
  );
}
